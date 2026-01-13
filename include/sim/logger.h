#pragma once

#include <fstream>
#include <string>

#include "plant/quad_state.h"
#include "common/types.h"

namespace gnc {

class Logger {
public:
  Logger(const std::string& csv_path);

  void writeHeader();
void log(const QuadStateTruth& x, const Eigen::Matrix<double,15,1> xhat,Vec4 T_cmd);

  bool isOpen() const;

private:
  std::ofstream file_;
  bool header_written_;
};

} // namespace gnc
