#include <string.h>

#include <algorithm>
// #include <iostream>
// using namespace std;
#include <numeric>
#include <string>
#include <vector>

#include "frame_struct.h"
frame_t* handleProcess(const std::string& s)
{
  static std::vector<uint8_t> vec_char;
  static const uint8_t SFLAG_L = FRAME_BEGIN_FLAG & 0xff;
  static const uint8_t SFLAG_H = (FRAME_BEGIN_FLAG >> 8) & 0xff;
  static const uint8_t EFLAG = FRAME_END_FLAG & 0xff;

  uint32_t frame_payload_len = 0;
  frame_t* pf = nullptr;
  std::vector<uint8_t>::iterator it;

  // cout << "vecChar before: " << vecChar.size() << endl;
  vec_char.insert(vec_char.end(), s.cbegin(), s.cend());
  // cout << "vecChar after: " << vecChar.size() << endl;

  if (vec_char.size() < 2)
  {
    // cerr << "data is not enough!" << endl;
    goto __finished;
  }

__find_header:
  for (it = vec_char.begin(); (*(it) != SFLAG_L) || (*(it + 1) != SFLAG_H);)
  {
    /* find sflag_l from [1:] first and next in [it+1:] */
    it = find(it + 1, vec_char.end(), SFLAG_L);
    /* sflag_l not found */
    if (it == vec_char.end())
    {
      /* clear all data and wait next data */
      vec_char.resize(0);
      // cerr << "frame head not found! wait more data." << endl;
      goto __finished;
    }
  }

  if (it != vec_char.begin())
  {
    std::vector<uint8_t>(it, vec_char.end()).swap(vec_char);
    // cerr << "frame move to first!" << endl;
  }

  if (vec_char.size() < sizeof(frame_t))
  {
    // cerr << "frame head data not enough now! wait more data." << endl;
    goto __finished;
  }

  pf = (frame_t*)&vec_char[0];
  frame_payload_len = pf->frame_head_.frame_data_len_ - FRAME_HEAD_DATA_SIZE;

  /* max frame payload size */
  if (frame_payload_len > 100 * 100)
  {
    // cerr << "frame head data invalid for large frame_payload_len." << endl;
    vec_char.pop_back();
    vec_char.pop_back();
    goto __find_header;
  }

  if (vec_char.size() < FRAME_HEAD_SIZE + frame_payload_len + FRAME_CHECKSUM_SIZE + FRAME_END_SIZE)
  {
    // cerr << "expected frame payload length: " << frame_payload_len << endl;
    // cerr << "frame payload data not enough now! wait more data." << endl;
    goto __finished;
  }

  {
    uint8_t check_sum =
        std::accumulate(vec_char.begin(), vec_char.begin() + FRAME_HEAD_SIZE + frame_payload_len, (uint8_t)0);

    if (check_sum != ((uint8_t*)pf)[FRAME_HEAD_SIZE + frame_payload_len] ||
        EFLAG != ((uint8_t*)pf)[FRAME_HEAD_SIZE + frame_payload_len + FRAME_CHECKSUM_SIZE])
    {
      // cerr << "src\tchecksum\ttail" << endl;
      // cerr << "data\t"
      //      << *(vecChar.begin() + FRAME_HEAD_SIZE + frame_payload_len) <<
      //      '\t'
      //      << *(vecChar.begin() + FRAME_HEAD_SIZE + frame_payload_len +
      //           FRAME_CHECKSUM_SIZE)
      //      << endl;
      // cerr << "data\t" << check_sum << '\t' << eflag << endl;
      vec_char.pop_back();
      vec_char.pop_back();
      goto __find_header;
    }
  }

  pf = (frame_t*)malloc(sizeof(frame_t) + frame_payload_len);
  memcpy(pf, &vec_char[0], sizeof(frame_t) + frame_payload_len);

  std::vector<uint8_t>(vec_char.begin() + FRAME_HEAD_SIZE + frame_payload_len + FRAME_CHECKSUM_SIZE + FRAME_END_SIZE,
                       vec_char.end())
      .swap(vec_char);
  return pf;

__finished:
  return nullptr;
}