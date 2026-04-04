/*
 *  RunInterface.cpp
 *
 *  Created on: April 4, 2026
 *      Author: Nathan (MacKante)
 */

#include "main_system.hpp"
#include "UARTDriver.hpp"
#include "RunInterface.hpp"

extern "C" {
    void run_interface()
    {
        run_main();
    }

    void cpp_USART1_IRQHandler()
    {
        Driver::uart1.HandleIRQ_UART();
    }
}


