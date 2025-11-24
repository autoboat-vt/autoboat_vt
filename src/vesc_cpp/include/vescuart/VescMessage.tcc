template <typename T>
void VescMessage::add(T data)
{
    for (int i = sizeof(T) - 1; i >= 0; i--)
    {
        push_back(static_cast<uint8_t>((data >> (8 * i)) & 0xFF));
    }
}

template <typename T, typename U>
void VescMessage::add(T data, float scale)
{
    add<U>(data * scale);
}

template <typename T>
T VescMessage::get()
{
    // write data into byte array
    uint8_t data[sizeof(T)];
    for (int i = sizeof(T) - 1; i >= 0; i--)
    {
        data[i] = at(0);
        erase(begin());
    }

    // reinterpret data pointer and return its value
    T* dataPtr = reinterpret_cast<T*>(data);
    return *dataPtr;
}

template <typename T, typename U>
T VescMessage::get(float scale)
{
    return static_cast<T>(get<U>()) / scale;
}
