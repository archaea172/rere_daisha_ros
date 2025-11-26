#pragma once

#include <smacc2/smacc.hpp>

#include "orthogonals/or_timer.hpp"
#include "orthogonals/or_keyboard.hpp"

namespace sm_rere_daisha 
{
    namespace states
    {
        struct StIdle;
        struct StActivate;
    }
}

using namespace smacc2;

namespace sm_rere_daisha
{
    struct SmRereDaisha
    : public SmaccStateMachineBase<SmRereDaisha, states::StIdle>
    {
        using SmaccStateMachineBase::SmaccStateMachineBase;

        void onInitialize() override {
            this->createOrthogonal<orthogonals::OrTimer>();
            this->createOrthogonal<orthogonals::OrKeyboard>();
        }     
    };
}
#include "states/st_idle.hpp"
#include "states/st_active.hpp"