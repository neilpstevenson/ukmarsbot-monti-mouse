#include "mbed.h"
#include "gpio_irq_handler.h"

/*
    The RP2040 can only register one function for GPIO interrupts.
    This header creates an interface using a jump table,
    allowing the user to attach pin and core specific callback functions.

    You may

    #define DEBUG_IRQ_DURATION_PIN 11 //for example
    To debug the duration of your ISR with a logic analyser.
    This will switch the pin high during the ISR and low otherwise.

    and

    #define HIGHEST_INTERRUPT_PIN 15 //for example
    To safe some memory in the jump table,
    but disabling interrupts on pins higher than specified.
*/

void cgpio_irq_handler::gpio_callback(uint gpio, uint32_t events)
{
#ifdef DEBUG_IRQ_DURATION_PIN
    gpio_put(DEBUG_IRQ_DURATION_PIN, true);
#endif

    isr_list[get_core_num()][gpio](gpio, events);

#ifdef DEBUG_IRQ_DURATION_PIN
    gpio_put(DEBUG_IRQ_DURATION_PIN, false);
#endif
}

cgpio_irq_handler::cgpio_irq_handler()
{
    for (int c = 0; c < NUM_CORES; ++c)
    {
        for (int p = 0; p <= HIGHEST_INTERRUPT_PIN; ++p)
        {
            isr_list[c][p] = do_nothing;
        }
    }

#ifdef DEBUG_IRQ_DURATION_PIN
    gpio_init(DEBUG_IRQ_DURATION_PIN);
    gpio_set_dir(DEBUG_IRQ_DURATION_PIN, GPIO_OUT);
#endif
}


void cgpio_irq_handler::register_interrupt(uint gpio, irq_event event, gpio_irq_callback_t callback)
{
    if(gpio > HIGHEST_INTERRUPT_PIN)
        return;
    if (num_registered_isrs++ == 0)
        gpio_set_irq_enabled_with_callback(gpio, event, true, &gpio_callback);
    else
        gpio_set_irq_enabled(gpio, event, true);
    
    isr_list[get_core_num()][gpio] = callback;
}

void cgpio_irq_handler::unregister_interrupt(uint gpio)
{
    if (isr_list[get_core_num()][gpio] != do_nothing)
    {
        gpio_set_irq_enabled(gpio, irq_event::none, false);
        isr_list[get_core_num()][gpio] = do_nothing;
        num_registered_isrs--;
    }
}

cgpio_irq_handler& irq_handler = cgpio_irq_handler::instance();
