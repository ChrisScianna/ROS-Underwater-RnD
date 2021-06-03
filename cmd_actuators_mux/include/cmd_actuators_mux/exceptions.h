#ifndef CMD_ACTUATORS_MUX_EXCEPTIONS_H_
#define CMD_ACTUATORS_MUX_EXCEPTIONS_H_

#include <exception>


namespace cmd_actuator_mux {


class FileNotFoundException: public std::runtime_error {
public:
  FileNotFoundException(const std::string& msg)
        : std::runtime_error(msg) {}
        virtual ~FileNotFoundException() throw() {}
};

class EmptyCfgException: public std::runtime_error {
public:
  EmptyCfgException(const std::string& msg)
        : std::runtime_error(msg) {}
        virtual ~EmptyCfgException() throw() {}
};

class YamlException: public std::runtime_error {
public:
  YamlException(const std::string& msg)
        : std::runtime_error(msg) {}
        virtual ~YamlException() throw() {}
};

}  //  namespace namespace cmd_actuator_mux

#endif /* CMD_ACTUATORS_MUX_EXCEPTIONS_H_ */
