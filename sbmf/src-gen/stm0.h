#ifndef ROBOCALC_STATEMACHINES_STM0_H_
#define ROBOCALC_STATEMACHINES_STM0_H_

#include "RoboCalcAPI/StateMachine.h"

#ifndef ROBOCALC_THREAD_SAFE
#define THREAD_SAFE_ONLY(x)
#else
#include <mutex>
#define THREAD_SAFE_ONLY(x) x
#endif

#include "RP1.h"
#include "RoboCalcAPI/Timer.h"
#include "Functions.h"
#include "DataTypes.h"
#include <assert.h>
#include <set>
#include "disableHV.h"
#include "checkLimits.h"
#include "supplyVoltCheck.h"

using namespace robocalc;
using namespace robocalc::functions;

template<typename ControllerType>
class stm0_StateMachine : public StateMachine<RP1, ControllerType, stm0_StateMachine<ControllerType>>
{
	public:
		ControllerType& controller;
		double ActualHV;
	public:
		class S0_State_t : public robocalc::State<ControllerType>
		{
			public:
				explicit S0_State_t(ControllerType& _controller, stm0_StateMachine<ControllerType>* topLevel) 
					: robocalc::State<ControllerType>(_controller)
				{
				}
				
				S0_State_t() = delete;
			
				void enter() override
				{
					this->controller.ActualHV = std::get<0>(this->controller.int_ActualHV_in._args);
					this->controller.int_ActualHV_in.reset();
				}
				
				void exit() override
				{
				}
				
				void during() override
				{
				}
		};
		
		S0_State_t s0_State;
		
		enum PossibleStates
		{
			s_s0,
			j_i0
		} currentState = j_i0; 

	public:
		stm0_StateMachine(RP1& _platform, ControllerType& _controller, stm0_StateMachine<ControllerType>* topLevel = nullptr) 
			: StateMachine<RP1, ControllerType, stm0_StateMachine<ControllerType>>(_platform, _controller, topLevel), controller{_controller},
			ActualHV(0),
			s0_State{_controller, this}
			{};
		stm0_StateMachine() = delete;
		~stm0_StateMachine() = default;
	
	
	
		void execute() override
		{
			while(tryTransitions());
		
			switch(currentState)
			{
			case s_s0:
			{
				s0_State.during();
				break;
			}
				default:
					break;
			}
		}
	
		bool tryTransitions() override
		{
			switch(currentState)
			{
				case s_s0:
				{
					{
						s0_State.exit();
						this->tryEmitActualHV_1(std::tuple<double>{this->ActualHV});
						this->tryEmitActualHV_2(std::tuple<double>{this->ActualHV});
						s0_State.enter();
						currentState = s_s0;
						return true;	
					}

					break;
				}
				case j_i0:
				{
					
					;
					currentState = s_s0;
					s0_State.enter();
					return true;
				}
				default:
					break;
			}
				
			return false;
		}
};
	
#endif
