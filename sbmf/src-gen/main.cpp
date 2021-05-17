#include <iostream>
#include <unistd.h>
#include "RobotImpl.h"
#include "ctrl0.h"
#include "ctrl1.h"
#include "ctrl2.h"
#include "ctrl3.h"

RobotImpl robot;

static constexpr long cycleDuration = 100e3;
			
int main(int argc, char** argv)
{
	Ctrl0 ctrl0{robot};
	Ctrl1 ctrl1{robot};
	Ctrl2 ctrl2{robot};
	Ctrl3 ctrl3{robot};
	
	while(true)
	{
		robot.Sense();
		
		// Signals to robot platform
		if(ctrl3.channels.actualHV_2_in.getSender() != nullptr)
		{
			std::cout<<"Signal Int_ActualHV emitted to robot"<<std::endl;
			robot.tryEmitInt_ActualHV(nullptr, ctrl3.channels.actualHV_2_in._args);
			ctrl3.channels.actualHV_2_in.reset();
		}
		if(ctrl0.channels.int_pwmSignal_in.getSender() != nullptr)
		{
			std::cout<<"Signal Int_pwmSignal emitted to robot"<<std::endl;
			robot.tryEmitInt_pwmSignal(nullptr, ctrl0.channels.int_pwmSignal_in._args);
			ctrl0.channels.int_pwmSignal_in.reset();
		}
		if(ctrl0.channels.currentState_in.getSender() != nullptr)
		{
			std::cout<<"Signal CurrentState emitted to robot"<<std::endl;
			robot.tryEmitCurrentState(nullptr, ctrl0.channels.currentState_in._args);
			ctrl0.channels.currentState_in.reset();
		}
		if(ctrl0.channels.ext_setPoint_in.getSender() != nullptr)
		{
			std::cout<<"Signal Ext_setPoint emitted to robot"<<std::endl;
			robot.tryEmitExt_setPoint(nullptr, ctrl0.channels.ext_setPoint_in._args);
			ctrl0.channels.ext_setPoint_in.reset();
		}
		
		// Signals from robot platform
		if(robot.shouldTriggerExt_setPoint())
			ctrl0.channels.tryEmitExt_setPoint(&robot, robot.getExt_setPointArgs());
			
		if(robot.shouldTriggerExt_pow24VStatus())
			ctrl2.channels.tryEmitExt_pow24VStatus(&robot, robot.getExt_pow24VStatusArgs());
			
		// Signals between controllers
		if(ctrl1.channels.int_DisableHV_in.getSender() != nullptr)
		{
			std::cout<<"Signal Int_DisableHV emitted from controller ctrl1 to ctrl0"<<std::endl;
			ctrl0.channels.tryEmitInt_DisableHV(&ctrl1, ctrl1.channels.int_DisableHV_in._args);
			ctrl1.channels.int_DisableHV_in.reset();
		}
			
		if(ctrl0.channels.int_underLimit_in.getSender() != nullptr)
		{
			std::cout<<"Signal Int_underLimit emitted from controller ctrl0 to ctrl1"<<std::endl;
			ctrl1.channels.tryEmitInt_underLimit(&ctrl0, ctrl0.channels.int_underLimit_in._args);
			ctrl0.channels.int_underLimit_in.reset();
		}
			
		if(ctrl0.channels.int_overLimit_in.getSender() != nullptr)
		{
			std::cout<<"Signal Int_overLimit emitted from controller ctrl0 to ctrl1"<<std::endl;
			ctrl1.channels.tryEmitInt_overLimit(&ctrl0, ctrl0.channels.int_overLimit_in._args);
			ctrl0.channels.int_overLimit_in.reset();
		}
			
		if(ctrl0.channels.int_ActualHV_in.getSender() != nullptr)
		{
			std::cout<<"Signal Int_ActualHV emitted from controller ctrl0 to ctrl3"<<std::endl;
			ctrl3.channels.tryEmitInt_ActualHV(&ctrl0, ctrl0.channels.int_ActualHV_in._args);
			ctrl0.channels.int_ActualHV_in.reset();
		}
			
		if(ctrl3.channels.actualHV_1_in.getSender() != nullptr)
		{
			std::cout<<"Signal ActualHV_1 emitted from controller ctrl3 to ctrl1"<<std::endl;
			ctrl1.channels.tryEmitInt_ActualHV(&ctrl3, ctrl3.channels.actualHV_1_in._args);
			ctrl3.channels.actualHV_1_in.reset();
		}
			
		if(ctrl2.channels.ext_pow24_2_in.getSender() != nullptr)
		{
			std::cout<<"Signal Ext_pow24_2 emitted from controller ctrl2 to ctrl1"<<std::endl;
			ctrl1.channels.tryEmitExt_pow24VStatus(&ctrl2, ctrl2.channels.ext_pow24_2_in._args);
			ctrl2.channels.ext_pow24_2_in.reset();
		}
			
		if(ctrl2.channels.ext_pow24_1_in.getSender() != nullptr)
		{
			std::cout<<"Signal Ext_pow24_1 emitted from controller ctrl2 to ctrl0"<<std::endl;
			ctrl0.channels.tryEmitExt_pow24VStatus(&ctrl2, ctrl2.channels.ext_pow24_1_in._args);
			ctrl2.channels.ext_pow24_1_in.reset();
		}
			
				
		ctrl0.Execute();
		ctrl1.Execute();
		ctrl2.Execute();
		ctrl3.Execute();
		
		robot.Actuate();
		
		#ifdef ROBOCALC_INTERACTIVE
		
		std::cout<<"Looped. Press enter for next iteration or q to quit."<<std::endl;
		char c;
		bool processingInput = true;
		while(processingInput)
		{
			std::cin.get(c);
			switch(c)
			{
				case '\n':
				{
					processingInput = false;
					break;
				}
				case 'q':
				case 'Q':
				{
					std::cout<<"Exiting"<<std::endl;
					return 0;
					break;
				}
				default:
					break;
			}
		}
		
		#else
			usleep(cycleDuration);				
		#endif
	}
	
	return 0;
}
