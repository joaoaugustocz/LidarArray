#ifndef VECTOR_H
#define VECTOR_H

#include <Arduino.h>

template<typename T>
class Vector {
public:
    Vector();
    ~Vector();

    void push_back(const T& value);
    void pop_back();
    T& at(size_t index);
    const T& at(size_t index) const;
    size_t size() const;
    bool empty() const;
    void clear();
    void resize(size_t newSize);
    void resize(size_t newSize, const T& value);

    T& operator[](size_t index);
    const T& operator[](size_t index) const;

private:
    T* data;
    size_t capacity;
    size_t count;

    void expandCapacity();
};

template<typename T>
Vector<T>::Vector() : data(nullptr), capacity(0), count(0) {}

template<typename T>
Vector<T>::~Vector() {
    delete[] data;
}

template<typename T>
void Vector<T>::push_back(const T& value) {
    if (count >= capacity) {
        expandCapacity();
    }
    data[count++] = value;
}

template<typename T>
void Vector<T>::pop_back() {
    if (count > 0) {
        --count;
    }
}

template<typename T>
T& Vector<T>::at(size_t index) {
    return data[index];
}

template<typename T>
const T& Vector<T>::at(size_t index) const {
    return data[index];
}

template<typename T>
size_t Vector<T>::size() const {
    return count;
}

template<typename T>
bool Vector<T>::empty() const {
    return count == 0;
}

template<typename T>
void Vector<T>::clear() {
    count = 0;
}

template<typename T>
void Vector<T>::resize(size_t newSize) {
    if (newSize > capacity) {
        T* newData = new T[newSize];
        for (size_t i = 0; i < count; ++i) {
            newData[i] = data[i];
        }
        delete[] data;
        data = newData;
        capacity = newSize;
    }
    count = newSize;
}

template<typename T>
void Vector<T>::resize(size_t newSize, const T& value) {
    if (newSize > capacity) {
        T* newData = new T[newSize];
        for (size_t i = 0; i < count; ++i) {
            newData[i] = data[i];
        }
        for (size_t i = count; i < newSize; ++i) {
            newData[i] = value;
        }
        delete[] data;
        data = newData;
        capacity = newSize;
    }
    count = newSize;
}

template<typename T>
T& Vector<T>::operator[](size_t index) {
    return data[index];
}

template<typename T>
const T& Vector<T>::operator[](size_t index) const {
    return data[index];
}

template<typename T>
void Vector<T>::expandCapacity() {
    size_t newCapacity = capacity == 0 ? 1 : capacity * 2;
    T* newData = new T[newCapacity];
    for (size_t i = 0; i < count; ++i) {
        newData[i] = data[i];
    }
    delete[] data;
    data = newData;
    capacity = newCapacity;
}

#endif // VECTOR_H
