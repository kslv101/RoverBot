#include "RoverBot.h"
#include <functional>
#include <map>
#include <iostream>

// Возможные состояния объекта
enum E_STATES
{
    TARGET_SELECTION = 0,
    PLANING = 1,
    MOVING = 2,
    WAIT = 3,
    REALSENCE = 4,
    READ_UPR = 5,
    SEND_UPR = 6
};

// Объект (тележка)
typedef struct Robo
{
    // Координаты
    int x;
    int y;

    // Переходы состояний
    bool start;
    bool planing_imposible;
    bool moving_ok;
    bool find_obstacle;
    bool find_target;
    bool mission_complete;

    // Триггеры
    struct triggers
    {
        bool arduino_error;
        bool realsense_error;
        bool correction_error;
    };
    triggers tg;

    // Глобальный стоп, для чего-нибудь пригодится
    bool GLOBAL_STOP = true;
} Robo;

// Функция обработки триггеров
E_STATES trigger(E_STATES our_state, Robo* cart)
{
    if ((cart->tg.arduino_error) || (cart->tg.realsense_error) || (cart->tg.correction_error))
    {
        return TARGET_SELECTION;
    }
    else
    {
        return our_state;
    }
};

// Функции состояний
E_STATES f_selection(Robo* cart)
{
    if (cart->start)
    {
        return PLANING;
    }
    else
    {
        return TARGET_SELECTION;
    }
};

E_STATES f_planing(Robo* cart)
{
    if (cart->planing_imposible)
    {
        return TARGET_SELECTION;
    }
    else
    {
        return MOVING;
    }
};

E_STATES f_moving(Robo* cart)
{
    return WAIT;
};

E_STATES f_wait(Robo* cart)
{
    if (cart->moving_ok)
    {
        return REALSENCE;
    }
    else
    {
        return WAIT;
    }
};

E_STATES f_realsence(Robo* cart)
{
    if (cart->find_obstacle)
    {
        return PLANING;
    }
    else if (cart->find_target)
    {
        return READ_UPR;
    }
    else
    {
        return MOVING;
    }
};

E_STATES f_read_upr(Robo* cart)
{
    if (cart->mission_complete)
    {
        return TARGET_SELECTION;
    }
    else
    {
        return SEND_UPR;
    }
};

E_STATES f_send_upr(Robo* cart)
{
    return READ_UPR;
};

// Тип функций для подачи их в словарь
typedef std::function<E_STATES(Robo* cart)> t_func;

int main()
{
    // Создание экземпляра объекта
    Robo cart;
    // Текущее состояние (Нулевое для удобства, сейчас это TARGET_SELECT)
    E_STATES our_state = (E_STATES)0;

    // Словарь состояний и функций переходов
    std::map<E_STATES, t_func> states;
    // Его заполнение
    states[TARGET_SELECTION] = f_selection;
    states[PLANING] = f_planing;
    states[MOVING] = f_moving;
    states[WAIT] = f_wait;
    states[REALSENCE] = f_realsence;
    states[READ_UPR] = f_read_upr;
    states[SEND_UPR] = f_send_upr;

    // Цикл перехода из состояния в состояние
    while (cart.GLOBAL_STOP)
    {
        our_state = trigger(our_state, &cart);

        auto _s = states.find(our_state);

        if (_s == states.end())
        {
            // Состояние не найдено
            cart.GLOBAL_STOP = false;
            break;
        }
        else
        {
            our_state = states[our_state](&cart);
        }
    }

    return 0;
}
