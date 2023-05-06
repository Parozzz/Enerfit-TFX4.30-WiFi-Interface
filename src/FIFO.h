#ifndef FIFO_H
#define FIFO_H

#include <Arduino.h>

template <typename T>
class FIFO
{
public:
    FIFO(T *buffer, size_t bufferLen)
    {
        _buffer = buffer;
        _bufferLen = bufferLen;
    }

    T read()
    {
        return _empty ? 0 : _buffer[_firstFullIndex];
    }
    
    T get()
    {
        if (_empty)
        {
            return 0;
        }
        _full = false;
        
        uint16_t itemIndex = _firstFullIndex;

        _firstFullIndex = (_firstFullIndex + 1) % _bufferLen;
        if (_firstFullIndex == _nextEmptyIndex)
        {
            _empty = true;
            _firstFullIndex = _nextEmptyIndex = 0;
        }

        return _buffer[itemIndex];
    }

    bool insert(T value)
    {
        if (_full)
        {
            return false;
        }
        _empty = false;

        _buffer[_nextEmptyIndex] = value;

        _nextEmptyIndex = (_nextEmptyIndex + 1) % _bufferLen;
        _full = (_nextEmptyIndex == _firstFullIndex);
        return true;
    }

    bool isEmpty()
    {
        return _empty;
    }

    bool isFull()
    {
        return _full;
    }

    uint16_t getDataCount()
    {
        if (_empty)
        {
            return 0;
        }

        return _nextEmptyIndex < _firstFullIndex
                   ? _bufferLen - (_firstFullIndex - _nextEmptyIndex)
                   : _nextEmptyIndex - _firstFullIndex;
    }

protected:
    size_t _bufferLen;
    T *_buffer;

    bool _empty = true;
    bool _full = false;
    uint16_t _firstFullIndex;
    uint16_t _nextEmptyIndex;
};

#endif