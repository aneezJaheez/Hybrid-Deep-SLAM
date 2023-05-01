#ifndef RECOGNITION_MESSAGE_H
#define RECOGNITION_MESSAGE_H

#include <iostream>
#include <cstring>
#include <opencv2/core/core.hpp>
// #include <mutex>

namespace ORB_SLAM2
{
  class RecognitionMessage
  {
    public:
      RecognitionMessage();
      RecognitionMessage(bool _flag, float _depth, std::string _name);
      RecognitionMessage(const RecognitionMessage &recog);
      void SetWorldPos(const std::vector<float> &Pos);
      const std::vector<float> GetWorldPos();
      bool flag;
      float depth;
      std::string name;
      std::vector<float> worldPos;
      ~RecognitionMessage();

      bool operator < (const RecognitionMessage &c) const
      {
        // std::cout << this->name << " < " << c.name << " == " << (this->name.compare(c.name) < 0) << std::endl;
        return (this->name.compare(c.name) < 0);
      }
      // bool operator == (const RecognitionMessage &c) const
      // {
      //   // std::cout << this->name << " < " << c.name << " == " << (this->name.compare(c.name) < 0) << std::endl;
      //   return (this->name.compare(c.name) == 0);
      // }
      // static std::mutex GlobalMutex;
  };
}
#endif // RECOGNITION_MESSAGE_H
