#ifndef __ROBOT_IMPL_H__
#define __ROBOT_IMPL_H__

#include "RP1.h"

#include <set>
#include <algorithm>
#include <iostream>

class RobotImpl : public RP1
{
public:
		
	bool shouldTriggerExt_pow24VStatus() override
	{
		std::cout<<"Should trigger event 'Ext_pow24VStatus'? (y/n)";
		char trigger;
		
		do { std::cin >> trigger; } while((trigger != 'y') && (trigger != 'n'));
		
		return trigger == 'y';
	}
	
	std::tuple<Power> getExt_pow24VStatusArgs() override
	{
		Power ret;
		{
			static const std::set<std::string> possibleValues = {"On", "Off"};
								
			std::string argString;
			
			do
			{
				std::cout<<"Enter value for argument (On, Off):";
				std::cin>>argString;
			} while(std::find(possibleValues.begin(), possibleValues.end(), argString) == possibleValues.end());
			
			if (argString == "On") ret = On;
			if (argString == "Off") ret = Off;
		}
		return ret;
	};
		
	bool shouldTriggerExt_setPoint() override
	{
		std::cout<<"Should trigger event 'Ext_setPoint'? (y/n)";
		char trigger;
		
		do { std::cin >> trigger; } while((trigger != 'y') && (trigger != 'n'));
		
		return trigger == 'y';
	}
	
	std::tuple<double> getExt_setPointArgs() override
	{
		double ret;
		std::cout<<"Enter value for argument (double):";
		std::cin>>ret;
		return ret;
	};
	
	void tryEmitCurrentState(void* sender, std::tuple<State> args) override
	{
		std::cout<<"Emitting signal CurrentState"<<std::endl;
	}
	
	
	void tryEmitInt_pwmSignal(void* sender, std::tuple<Power> args) override
	{
		std::cout<<"Emitting signal Int_pwmSignal"<<std::endl;
	}
	
	
	void tryEmitInt_ActualHV(void* sender, std::tuple<double> args) override
	{
		std::cout<<"Emitting signal Int_ActualHV"<<std::endl;
	}
	
	
	
	void tryEmitExt_setPoint(void* sender, std::tuple<double> args) override
	{
		std::cout<<"Emitting signal Ext_setPoint"<<std::endl;
	}
	
};

#endif
