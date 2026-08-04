#include "observer.h"

#if (defined (FOC_WITHOUT_SENSOR))
    void ObserverSMO(Motor_t *motor)
    {

    }
    void PLL(Motor_t *motor)
    {

    }
#elif (defined (FOC_WITH_SENSOR))
// observer method
#endif