#ifndef GPIO_IRQ_HANDLER
#define GPIO_IRQ_HANDLER

//#include "pico/stdlib.h"
#include "pico/sync.h"

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



//https://de.wikibooks.org/wiki/C%2B%2B-Programmierung:_Entwurfsmuster:_Singleton#Makro_(Stack)
#define DEF_SINGLETON( NAME )   \
public:                         \
    static NAME& instance()     \
    {                           \
        static NAME _instance;  \
        return _instance;       \
    }                           \
private:                        \
    NAME();                     \
    NAME( const NAME& );


#ifndef HIGHEST_INTERRUPT_PIN
#define HIGHEST_INTERRUPT_PIN 28
#endif


class cgpio_irq_handler
{
DEF_SINGLETON(cgpio_irq_handler)

public:
    enum irq_event
    {
        none = 0x0,
        low = GPIO_IRQ_LEVEL_LOW,
        high = GPIO_IRQ_LEVEL_HIGH,
        fall = GPIO_IRQ_EDGE_FALL,
        rise = GPIO_IRQ_EDGE_RISE,
        change = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL
    };

    void register_interrupt(uint gpio, irq_event event, gpio_irq_callback_t callback);
    void unregister_interrupt(uint gpio);

private:
    int num_registered_isrs = 0;
    inline static gpio_irq_callback_t isr_list[NUM_CORES][HIGHEST_INTERRUPT_PIN + 1];

    static void do_nothing(uint gpio, uint32_t events) {}
    static void gpio_callback(uint gpio, uint32_t events);
};

extern cgpio_irq_handler& irq_handler;

#endif
