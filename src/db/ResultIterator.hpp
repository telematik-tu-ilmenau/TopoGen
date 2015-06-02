#ifndef RESULTITERATOR_HPP
#define RESULTITERATOR_HPP

template <class Result>
class ResultIterator {
   public:
    ResultIterator() : _rowAvailable(false) {}

    bool hasNext() { return _rowAvailable; }

    virtual Result getNext() = 0;

    virtual ~ResultIterator() {}
   protected:
    bool _rowAvailable;
};

#endif  // RESULTITERATOR_HPP
