
#ifndef morslib_h
#define morslib_h

// const int buffLen = 10;

class morslib
{
public:
    morslib(int pinNumber, unsigned int ditTimeMS);
    void handle();
    void begin();
    void queue(char znak, bool priority=0);
    char getbuf(int i);

private:
    void queue0(char znak, bool priority=0);
    bool dit();
    bool dah();
    bool pause();
    bool blinkIndex(int index);
    int char2index(char znak);
    int _pinNumber;
    unsigned int _ditTimeMS;
    unsigned int _dahTimeMS;
    //int _number;
    unsigned long _startTime;
    bool _startTimeSet;
   // bool _done;
    int _indexCol;
    //int _indexRow;
    char _bufor[128];
};
#endif