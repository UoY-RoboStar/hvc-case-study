#ifndef ROBOCALC_ROBOT_RP1_H_
#define ROBOCALC_ROBOT_RP1_H_

#include "DataTypes.h"

class RP1 {
public:
	RP1() = default; 
	virtual ~RP1() = default;
	virtual void Sense() {};
	virtual void Actuate() {};
	

	virtual bool shouldTriggerExt_pow24VStatus() = 0;
	virtual std::tuple<Power> getExt_pow24VStatusArgs() = 0;

	virtual bool shouldTriggerExt_setPoint() = 0;
	virtual std::tuple<double> getExt_setPointArgs() = 0;

	virtual void tryEmitCurrentState(void* sender, std::tuple<State> args) = 0;

	virtual void tryEmitInt_pwmSignal(void* sender, std::tuple<Power> args) = 0;

	virtual void tryEmitInt_ActualHV(void* sender, std::tuple<double> args) = 0;

	virtual void tryEmitExt_setPoint(void* sender, std::tuple<double> args) = 0;
};

#endif
