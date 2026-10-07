// Drive the real replay through a late steady-budget failure without adding a
// production fault switch. Frozen ROS acquisition time is left unchanged.
#define main curve_backend_replay_main
#include "../src/curve_backend_replay.cpp"
#undef main
#include <thread>

class CorrectionDeadlineBuffer : public std::streambuf {
 public:
  explicit CorrectionDeadlineBuffer(std::streambuf* output) : output_(output) {}
 protected:
  std::streamsize xsputn(const char* text,std::streamsize size) override {
    const auto written=output_->sputn(text,size);
    for(std::streamsize i=0;i<size;++i) {
      line_.push_back(text[i]);
      if(text[i]!='\n') continue;
      if(!expired_ && line_.find("correction prepared repairs=")!=std::string::npos) {
        expired_=true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1600));
      }
      line_.clear();
    }
    return written;
  }
  int overflow(int value) override {
    if(traits_type::eq_int_type(value,traits_type::eof())) return traits_type::not_eof(value);
    const char ch=traits_type::to_char_type(value);
    return xsputn(&ch,1)==1 ? value : traits_type::eof();
  }
  int sync() override { return output_->pubsync(); }
 private:
  std::streambuf* output_;
  std::string line_;
  bool expired_=false;
};

int main(int argc,char** argv) {
  auto* original=std::cout.rdbuf();
  CorrectionDeadlineBuffer deadline(original);
  std::cout.rdbuf(&deadline);
  const int result=curve_backend_replay_main(argc,argv);
  std::cout.rdbuf(original);
  return result;
}
