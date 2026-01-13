#include "sim/logger.h"

#include <iomanip>

namespace gnc {

Logger::Logger(const std::string& csv_path)
  : file_(),
    header_written_(false)
{
  file_.open(csv_path.c_str(), std::ios::out | std::ios::trunc);
}

bool Logger::isOpen() const {
  return file_.is_open();
}

void Logger::writeHeader() {
  if (!file_.is_open()) {
    return;
  }
  if (header_written_) {
    return;
  }

  file_ << "t,"
        << "n,e,d,"
        << "vn,ve,vd,"
        << "phi,theta,psi,"
        << "p,q,r,"
		<< "phiE,thetaE,psiE,"
		<< "nE,eE,dE,"
		<< "vnE,veE,vdE,"
		<< "T1,T2,T3,T4"
        << "\n";

  header_written_ = true;
}

void Logger::log(const QuadStateTruth& x,const Eigen::Matrix<double,15,1> xhat,Vec4 T_cmd) {
  if (!file_.is_open()) {
    return;
  }
  if (!header_written_) {
    writeHeader();
  }

  file_ << std::fixed << std::setprecision(6);

  file_ << x.t << ","
        << x.pos(0) << "," << x.pos(1) << "," << x.pos(2) << ","
        << x.vel(0) << "," << x.vel(1) << "," << x.vel(2) << ","
        << x.euler(0) << "," << x.euler(1) << "," << x.euler(2) << ","
        << x.omega_b(0) << "," << x.omega_b(1) << "," << x.omega_b(2) << ","
		<< xhat(0) << "," << xhat(1) << "," << xhat(2) << ","
		<< xhat(3) << "," << xhat(4) << "," << xhat(5) << ","
		<< xhat(6) << "," << xhat(7) << "," << xhat(8) << ","
		<< T_cmd(0) << "," << T_cmd(1) << "," << T_cmd(2) << "," << T_cmd(3) << ","
        << "\n";
}

} // namespace gnc
