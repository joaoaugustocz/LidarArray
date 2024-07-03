#ifndef ARRAYLIST_H
#define ARRAYLIST_H

#include <Arduino.h>

template <typename T>
class ArrayList {
private:
    T* data;
    size_t capacity;
    size_t size;

    void resize(size_t newCapacity);

public:
    ArrayList(size_t initialCapacity = 10);
    ~ArrayList();

    void add(T value);
    void removeAt(size_t index);
    T get(size_t index) const;
    size_t getSize() const;
    size_t getCapacity() const;
    void clear();

    T& operator[](size_t index);
    const T& operator[](size_t index) const;
};

template <typename T>
ArrayList<T>::ArrayList(size_t initialCapacity) {
    capacity = initialCapacity;
    size = 0;
    data = new T[capacity];
}

template <typename T>
ArrayList<T>::~ArrayList() {
    delete[] data;
}

template <typename T>
void ArrayList<T>::resize(size_t newCapacity) {
    T* newData = new T[newCapacity];
    for (size_t i = 0; i < size; i++) {
        newData[i] = data[i];
    }
    delete[] data;
    data = newData;
    capacity = newCapacity;
}

template <typename T>
void ArrayList<T>::add(T value) {
    if (size >= capacity) {
        resize(capacity * 2);
    }
    data[size++] = value;
}

template <typename T>
void ArrayList<T>::removeAt(size_t index) {
    if (index >= size) return;
    for (size_t i = index; i < size - 1; i++) {
        data[i] = data[i + 1];
    }
    size--;
}

template <typename T>
T ArrayList<T>::get(size_t index) const {
    if (index >= size) return T();
    return data[index];
}

template <typename T>
size_t ArrayList<T>::getSize() const {
    return size;
}

template <typename T>
size_t ArrayList<T>::getCapacity() const {
    return capacity;
}

template <typename T>
void ArrayList<T>::clear() {
    size = 0;
}

template <typename T>
T& ArrayList<T>::operator[](size_t index) {
    return data[index];
}

template <typename T>
const T& ArrayList<T>::operator[](size_t index) const {
    return data[index];
}

#endif
