#ifndef INPUT_MESSAGE_PARSER_HPP
#define INPUT_MESSAGE_PARSER_HPP

#include "errno.h"

/*
      DEPRECATED. BINARY PROTOCOL USED INSTEAD
*/


class InputMessageParser {
public:
  InputMessageParser(String& input_msg)
    : input_msg_{input_msg}
  {}

  ~InputMessageParser() = default;
  InputMessageParser(const InputMessageParser&) = delete;
  InputMessageParser(InputMessageParser&&) = delete;
  InputMessageParser& operator=(const InputMessageParser&) = delete;
  InputMessageParser& operator=(InputMessageParser&&) = delete;

  bool parseVelocityMessage(double& r_wheel_velocity, double& l_wheel_velocity) {
    // Absolute minimum length for a single component
    // (e.g. "rp0.0," or "lp0.0,")
    constexpr int kWheelCmdMinLength{6};

    // Absolute minimum length for whole message
    // (e.g. "rp0.0,lp0.0,")
    constexpr int kMsgMinLength{2 * kWheelCmdMinLength};

    index_ = 0;

    if(input_msg_.length() < kMsgMinLength) {
      return false;
    }
    if ('r' != input_msg_.charAt(index_++)) {
      return false;
    }
    if (!parseSignedDoubleValue(r_wheel_velocity)) {
      return false;
    }
    // Ensure remaining string length is enough to reasonably hold the left wheel command
    if (index_ >= input_msg_.length() || input_msg_.length() - index_ < kWheelCmdMinLength) {
      return false;
    }
    if ('l' != input_msg_.charAt(index_++)) {
      return false;
    }
    if (!parseSignedDoubleValue(l_wheel_velocity)) {
      return false;
    }
    // Ensure no trailing garbage characters exist past the final expected comma
    if (index_ != input_msg_.length()) {
      return false;
    }
    return true;
  }

  bool parseInitMessage(double& pid_control_rate, 
                        double& r_kp, double& r_ki, double& r_kd,
                        double& l_kp, double& l_ki, double& l_kd) {
    constexpr int kPidControlRateCmdMinLength{4};
    // e.g. rkp0.1, or lki0.1, or rkd0.1,
    constexpr int kWheelPidGainCmdMinLength{7};
    // Absolute minimum length for whole message
    // (e.g. "pr1,rkp0.1,rki0.1,rkd0.1,lkp0.1,lki0.1,lkd0.1,")
    constexpr int kMsgMinLength{kPidControlRateCmdMinLength + 6 * kWheelPidGainCmdMinLength};

    index_ = 0;

    if(input_msg_.length() < kMsgMinLength) {
      return false;
    }
    if ('p' != input_msg_.charAt(index_++) || 'r' != input_msg_.charAt(index_++)) {
      return false;
    }
    if (false == parseDoubleValue(pid_control_rate) || index_ >= input_msg_.length()) {
      return false;
    }
    int message_length_min_remainder{kMsgMinLength - kPidControlRateCmdMinLength};
    if (input_msg_.length() - index_ < message_length_min_remainder) {
      return false;
    }
    // Parse 6 PID value components, 3 per each wheel 
    constexpr char kWheelIds[] = "rl";
    constexpr char kPidIds[] = "pid";
    double* pid_values[] = {&r_kp, &r_ki, &r_kd, &l_kp, &l_ki, &l_kd};
    int pid_value_idx{0};
  
    for (size_t wheel_idx = 0; wheel_idx < 2; wheel_idx++) {
      for (size_t pid_idx = 0; pid_idx < 3; pid_idx++) {
        const bool valid_cmd_id = 
          kWheelIds[wheel_idx] == input_msg_.charAt(index_++) &&
          'k' == input_msg_.charAt(index_++) &&
          kPidIds[pid_idx] == input_msg_.charAt(index_++);
        if (!valid_cmd_id) {
          return false;
        }
        if (false == parseDoubleValue(*pid_values[pid_value_idx++]) || index_ > input_msg_.length()) {
          return false;
        }
        message_length_min_remainder -= kWheelPidGainCmdMinLength;
        if (input_msg_.length() - index_ < message_length_min_remainder) {
          return false;
        }
      }
    }

    // Ensure no trailing garbage characters exist past the final expected comma
    if (index_ != input_msg_.length()) {
      return false;
    }

    return true;
  }

private:
    bool parseSignedDoubleValue(double& value) {
      bool result{false};

      if ('n' == input_msg_.charAt(index_)) {
        const int negative_index = index_;
        input_msg_.setCharAt(negative_index, '-');
        result = parseDoubleValue(value);
        input_msg_.setCharAt(negative_index, 'n');
      } else if ('p' == input_msg_.charAt(index_)) {
        index_ += 1;
        result = parseDoubleValue(value);
      }

      return result;
    }

    bool parseDoubleValue(double& value) {
      errno = 0;
      char* value_end{nullptr};
      const char* value_start = input_msg_.c_str() + index_;
      value = strtod(value_start, &value_end);
      if (value_end == value_start || errno != 0 || ',' != *value_end) {
        return false;
      }
      // Advance index past the parsed double numbers
      // AND the trailing comma
      index_ += value_end - value_start + 1;
      return true;
    }

  String& input_msg_;
  int index_{0};
};

#endif // INPUT_MESSAGE_PARSER_HPP