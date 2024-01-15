#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace boost;

#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono;
using namespace std::literals::chrono_literals;

#define DEFAULT_USRT_PORT "/dev/ttyUSB1"
#define DEFAULT_READ_TIMEDOUT 200

#define LOG_LINE                                                                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    std::cout << "[" << duration_cast<milliseconds>(high_resolution_clock::now() - _start).count() * 0.001 << "]["     \
              << __LINE__ << "][" << std::this_thread::get_id() << ']' << std::endl;                                   \
  } while (0)
static std::chrono::time_point<std::chrono::high_resolution_clock> g_START = high_resolution_clock::now();

class Msa010
{
private:
  const char* serial_path_;
  std::size_t read_timedout_;
  asio::io_service ioc_;
  asio::serial_port sp_;
  asio::deadline_timer timeout_;
  // asio::executor_work_guard<asio::io_service::executor_type> _worker;
  std::unique_ptr<asio::io_service::work> worker_;
  std::thread ioc_run_thread_;

public:
  Msa010() : Msa010(DEFAULT_USRT_PORT)
  {
  }
  Msa010(const char* __serial_path)
    : serial_path_(__serial_path)
    , read_timedout_(DEFAULT_READ_TIMEDOUT)
    , ioc_()
    , sp_(ioc_)
    , timeout_(ioc_)
    ,
    // _worker(asio::make_work_guard(_ioc)),
    worker_(new asio::io_service::work(ioc_))
    , ioc_run_thread_(std::bind([](asio::io_service& _ioc) { _ioc.run(); }, std::ref(ioc_)))
  {
  }

  ~Msa010()
  {
    worker_.reset();
    ioc_run_thread_.join();
  };

  inline bool isConnected()
  {
    return sp_.is_open();
  }

  void keepConnect(const std::function<bool(void)>& f)
  {
    boost::system::error_code ec;
    if (sp_.is_open())
      return;
    do
    {
      ec = tryConnect();
      // std::cout << "[INFO] Try connect to " << _serial_path << "..."
      //           << " Result: " << ec.message() << std::endl;
    } while (f() && ec);
  };

  void keepConnect()
  {
    keepConnect([] {
      std::this_thread::sleep_for(1s);
      return true;
    });
    // boost::system::error_code ec;
    // if (_sp.is_open()) return;
    // do {
    //   ec = try_connect();
    //   // std::cout << "[INFO] Try connect to " << _serial_path << "..."
    //   //           << " Result: " << ec.message() << std::endl;
    //   std::this_thread::sleep_for(1s);
    // } while (ec);
  };

  void setReadTimedout(std::size_t timeout)
  {
    this->read_timedout_ = timeout;
  }

  template <typename MutableBufferSequence>
  std::size_t readTimedout(const MutableBufferSequence& buffers)
  {
    return readTimedout(buffers, read_timedout_);
  }

  template <typename MutableBufferSequence>
  std::size_t readTimedout(const MutableBufferSequence& buffers, std::size_t ms)
  {
    bool is_timeout = false;
    bool had_data = false;
    std::size_t transferred = 0;

    timeout_.expires_from_now(boost::posix_time::milliseconds(ms));
    timeout_.async_wait([this, &had_data, &is_timeout](const boost::system::error_code& error) {
      if (error)
      {
        switch (error.value())
        {
          case boost::asio::error::operation_aborted: /* Operation canceled */ {
            // Data was read and this timeout was canceled
            // std::cout << "[WARN][Timeout] Cancelled, Has Data..." <<
            // std::endl;
            had_data = true;
            return;
          }
          break;
          default:
            break;
        }
      }

      // timeout
      if (this->sp_.is_open())
      {
        // std::cout << "[WARN][Timeout] Was Fired, No Data..." << std::endl;
        this->sp_.cancel();  // will cause read_callback to fire with an error
      }
      is_timeout = true;
    });

    sp_.async_read_some(buffers,
                        [this, &transferred](const boost::system::error_code& error, std::size_t bytes_transferred) {
                          this->timeout_.cancel();  // will cause wait_callback to fire with an error
                          if (error)
                          {
                            // No data was read!
                            switch (error.value())
                            {
                              case boost::asio::error::eof: /* End of file */ {
                                /* disconnect */
                                // std::cout << "[WARN][Serial] End of file..." << std::endl;
                                this->sp_.close();
                              }
                              break;
                              case boost::asio::error::bad_descriptor: /* Bad file descriptor */ {
                                // std::cout << "[WARN][Serial] Bad file descriptor..." <<
                                // std::endl;
                                this->sp_.close();
                              }
                              break;
                            }
                            return;
                          }

                          transferred = bytes_transferred;
                        });

    while (!(is_timeout || had_data))
    {
      std::this_thread::sleep_for(20ms);
    }

    return transferred;
  }

  std::size_t readSome(std::string& s)
  {
#define MAX_SIZE ((25 * 25 + 22) * 4 * 4)
    std::size_t len;
    boost::system::error_code ec;

    s.reserve(MAX_SIZE);
    len = this->sp_.read_some(asio::buffer(s, MAX_SIZE), ec);
    if (ec)
      this->sp_.close();

    return len;
  }

  inline Msa010& operator<<(const std::string& s)
  {
    boost::system::error_code ec;
    this->sp_.write_some(asio::buffer(s), ec);
    if (ec)
      this->sp_.close();
    return *this;
  }

  inline Msa010& operator>>(std::string& s)
  {
#define MAX_SIZE ((25 * 25 + 22) * 4 * 4)
    static char b[MAX_SIZE];
    std::size_t len;

    s.clear();
    len = readTimedout(asio::buffer(b));
    if (len)
    {
      s.append(b, len);
    }

    return *this;
  }

  friend inline std::unique_ptr<Msa010>& operator<<(std::unique_ptr<Msa010>& a010, const std::string& s)
  {
    *a010 << s;
    return a010;
  }

  friend inline std::unique_ptr<Msa010>& operator>>(std::unique_ptr<Msa010>& a010, std::string& s)
  {
    *a010 >> s;
    return a010;
  }

private:
  boost::system::error_code tryConnect()
  {
    using sp = boost::asio::serial_port;

    boost::system::error_code ec;
    sp_.open(serial_path_, ec);
    if (ec)
      return ec;
    try
    {
      // baud rate
      sp_.set_option(sp::baud_rate(115200));
      // character size
      sp_.set_option(sp::character_size(8));
      // Parity check, can be serial_port::parity::none / odd / even.
      sp_.set_option(sp::parity(sp::parity::none));
      // Stop bit, can be serial_port::stop_bits::one / onepointfive /two
      sp_.set_option(sp::stop_bits(sp::stop_bits::one));
      // Flow control, which can be serial_port::flow_control::type, enum type, which can be none
      sp_.set_option(sp::flow_control(sp::flow_control::none));
    }
    catch (boost::system::system_error const& ex)
    {
      ec = ex.code();
      sp_.close();
    }
    return ec;
  }
};
