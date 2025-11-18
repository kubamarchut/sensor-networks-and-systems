#include "morslib.h"
#include "Arduino.h"

const int morsCharlen = 6;
const int codesArray[][morsCharlen] = {
    /*
    dot         =1
    dash        =2
    end filler  =0
    */
    {1, 2, 0, 0, 0, 0}, // a
    {2, 1, 1, 1, 0, 0}, // b
    {2, 1, 2, 1, 0, 0}, // c
    {2, 1, 1, 0, 0, 0}, // d
    {1, 0, 0, 0, 0, 0}, // e
    {1, 1, 2, 1, 0, 0}, // f
    {2, 2, 1, 0, 0, 0}, // g
    {1, 1, 1, 1, 0, 0}, // h
    {1, 1, 0, 0, 0, 0}, // i
    {1, 2, 2, 2, 0, 0}, // j
    {2, 1, 2, 0, 0, 0}, // k
    {1, 2, 1, 1, 0, 0}, // l
    {2, 2, 0, 0, 0, 0}, // m
    {2, 1, 0, 0, 0, 0}, // n
    {2, 2, 2, 0, 0, 0}, // o
    {1, 2, 2, 1, 0, 0}, // p
    {2, 2, 1, 2, 0, 0}, // q
    {1, 2, 1, 0, 0, 0}, // r
    {1, 1, 1, 0, 0, 0}, // s
    {2, 0, 0, 0, 0, 0}, // t
    {1, 1, 2, 0, 0, 0}, // u
    {1, 1, 1, 2, 0, 0}, // v
    {1, 2, 2, 0, 0, 0}, // w
    {2, 1, 1, 2, 0, 0}, // x
    {2, 1, 2, 2, 0, 0}, // y
    {2, 2, 1, 1, 0, 0}, // z
    {2, 2, 2, 2, 2, 0}, // 0
    {1, 2, 2, 2, 2, 0}, // 1
    {1, 1, 2, 2, 2, 0}, // 2
    {1, 1, 1, 2, 2, 0}, // 3
    {1, 1, 1, 1, 2, 0}, // 4
    {1, 1, 1, 1, 1, 0}, // 5
    {2, 1, 1, 1, 1, 0}, // 6
    {2, 2, 1, 1, 1, 0}, // 7
    {2, 2, 2, 1, 1, 0}, // 8
    {2, 2, 2, 2, 1, 0}, // 9
};

morslib::morslib(int pinNumber, unsigned int ditTimeMS)
{
    _pinNumber = pinNumber;
    _ditTimeMS = ditTimeMS;
    _dahTimeMS = ditTimeMS * 3;
    _startTimeSet = 0;
    _indexCol = 0;
}
void morslib::begin()
{
    pinMode(_pinNumber, OUTPUT);
    _bufor[0] = 0;
    /*_bufor[1] = 'b';
    _bufor[2] = 'c';
    _bufor[3] = 'd';
    _bufor[2] = 'd';*/
}

/*void morslib::handle()
{
    




    // Serial.println(char2index('d'));

    bool ender;
    if (wyblyskaj)
    {
        // ender = dah();
        if (blinkIndex(char2index(_bufor[0])))
        {
            wyblyskaj = 0;

            for (int i = 0; _bufor[i] != 0; i++) // TAK MOGĘ SIĘ ITEROWAĆ PO BUFORZE
            {
                Serial.print(_bufor[i]);
                Serial.println("");
                _bufor[i] = _bufor[i + 1];
            }
        }
    }
}*/

void morslib::handle()
{
    // if (_bufor[0] != 0)
    // {
    if (blinkIndex(char2index(_bufor[0])))
    {
        int i = 0;
        do
        {
            _bufor[i] = _bufor[i + 1];
            i++;
        } while (_bufor[i] != 0);
    }
    // }
    // _bufor[0] = 'a';
}

void morslib::queue(char znak, bool priority)
{
    if (!priority)
    {
        int i = 0;
        while (((_bufor[i] != 0) && (_bufor[i] != znak)))
        {
            // Serial.println(i);
            // Serial.println(int(_bufor[i]));
            i++;
        }
        _bufor[i] = znak;
    }
    else
    {
        if ((_bufor[0] != znak) && (_bufor[1] != znak) && (_bufor[2] != znak))
        {
            char temp1 = _bufor[1];
            char temp2;
            int i = 2;
            for (i; _bufor[i] != 0; i++)
            {
                temp2 = _bufor[i];
                _bufor[i] = temp1;
                temp1 = temp2;
            }
            _bufor[i] = temp1;
            _bufor[1] = znak;
        }
    }
}

int morslib::char2index(char znak)
{
    if (znak >= 97 && znak <= 122) // małe litery
    {
        znak = znak - 97;
    }
    else if (znak >= 65 && znak <= 90) // wielkie litery
    {
        znak = znak - 65;
    }
    else if (znak >= 48 && znak <= 57) // cyfry
    {
        znak = znak - 48 + 26;
    }
    else
    {
        return 999;
    }
    return znak;
}

bool morslib::blinkIndex(int indexRow)
{
    //_indexRow = indexRow;
    // Serial.println(codesArray[_indexRow][_indexCol]);

    if (indexRow == 999)
    {
        if (pause())
        {
            return 1;
        }
        return 0;
    }
    else
    {
        switch (codesArray[indexRow][_indexCol])
        {
        case 0:
            if (pause())
            {
                _indexCol = 0;
                // Serial.println("case0");
                return 1;
                break;
            }
            return 0;
            break;
        case 1:
            if (dit())
            {
                _indexCol++;
                // Serial.println("case1");
            }
            // _done = dit();

            return 0;
            break;
        case 2:
            if (dah())
            {
                _indexCol++;
                // Serial.println("case2");
            }
            //_done = dah();

            return 0;
            break;
        default:
            return 0;
        }
        
    }
}

bool morslib::dit()
{
    if (!_startTimeSet)
    {
        _startTimeSet = 1;
        _startTime = millis();
    }
    if (millis() - _startTime < _ditTimeMS)
    {
        digitalWrite(_pinNumber, 1);
    }
    if (millis() - _startTime >= _ditTimeMS)
    {
        digitalWrite(_pinNumber, 0);
        if (millis() - _startTime >= _ditTimeMS + _ditTimeMS)
        {
            _startTimeSet = 0;
            return 1;
        }
        else
            return 0;
    }
    else
        return 0;
}

bool morslib::dah()
{
    if (!_startTimeSet)
    {
        _startTimeSet = 1;
        _startTime = millis();
    }
    if (millis() - _startTime < _dahTimeMS)
    {
        digitalWrite(_pinNumber, 1);
    }
    if (millis() - _startTime >= _dahTimeMS)
    {
        digitalWrite(_pinNumber, 0);
        if (millis() - _startTime >= _dahTimeMS + _ditTimeMS)
        {
            _startTimeSet = 0;
            return 1;
        }
        else
            return 0;
    }
    else
        return 0;
}

bool morslib::pause()
{
    if (!_startTimeSet)
    {
        _startTimeSet = 1;
        _startTime = millis();
    }
    if (millis() - _startTime < _dahTimeMS)
    {
        digitalWrite(_pinNumber, 0);
        return 0;
    }
    else
    {
        _startTimeSet = 0;
        return 1;
    }
}

char morslib::getbuf(int i){
    return _bufor[i];
}
/*
void morslib::blink(int number)
{
    for (int i = 0; i < number; i++)
    {
        digitalWrite(_pinNumber, 1);
        delay(_ditTimeMS);
        digitalWrite(_pinNumber, 0);
        delay(_ditTimeMS);
    }
}*/