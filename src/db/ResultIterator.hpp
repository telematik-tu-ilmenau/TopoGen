#ifndef RESULTITERATOR_HPP
#define RESULTITERATOR_HPP

template <class Result>
class ResultIterator {
   public:
    bool hasNext() { return _rowAvailable; }

    virtual Result getNext() = 0;

   protected:
    bool _rowAvailable;
};

#endif  // RESULTITERATOR_HPP
