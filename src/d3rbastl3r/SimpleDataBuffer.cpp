/**
 * @author Igor Martens (d3rbastl3r)
 */

#ifndef _D3RBASTL3R__SIMPLE_DATA_BUFFER__CPP_
#define _D3RBASTL3R__SIMPLE_DATA_BUFFER__CPP_

#include <stdint.h>

template <uint8_t BUFFER_SIZE>
class SimpleDataBuffer {
    public:
        SimpleDataBuffer() : readPos(0), writePos(0), ready(false) {}

        /**
         * Add one byte to the buffer.
         * Ignore incoming data, if buffer is full or if data is marked as ready to send.
         * 
         * @data the byte which should be added to the buffer
         */
        void append2Buffer(uint8_t data) {
            // If data is ready to be transmitted, incoming data will be ignored
            if (ready) {
                return;
            }

            // If buffer is full, incoming data will be ignored
            if (writePos >= BUFFER_SIZE) {
                return;
            }

            buffer[writePos++] = data;
        }

        /**
         * Convert the 16 bit number to two string visualization and add the data to the buffer
         * 
         * @data number which should be added to the buffer as string
         */
        void append2BufferAsStr(uint16_t number) {
            uint16_t devider = 10000;

            // find the right size of the devider
            while ((number / devider) == 0 && devider >=  10) {
                devider /= 10;
            }

            uint16_t tempNumber = number;
            uint8_t numberPos = 0;
            for (; devider > 0; devider /= 10) {
                numberPos = tempNumber / devider;
                append2Buffer('0' + numberPos);
                tempNumber -= numberPos * devider;
            }
        }

        /**
         * Returns true if a next byte is available for transmission.
         * If the flag 'ready' is not set, this method will return false.
         */
        bool hasNext() {
            return readPos < writePos && ready;
        }

        /**
         * Get the next byte. It is recommended to check if the next byte is available or not.
         * In case of error, value 0x00 will be returned
         * 
         * The buffer will be resetted if the last byte is returned
         * 
         * @return the byte which can be transmitted
         */
        uint8_t getNext() {
            // If no data is available, 0x00 will be returned
            if (hasNext()) {
                return 0x00;
            }

            // Reset the buffer before sending the last byte
            if (readPos+1 == writePos) {
                reset();
            }

            return buffer[readPos++];
        }

        void setReady() {
            ready = true;
        }

        bool isReady() {
            return ready;
        }

        /**
         * Reset the buffer.
         * Can be executed manually or will be executed automatically after reading the last byte from buffer
         */
        void reset() {
            readPos = 0;
            writePos = 0;
            ready = false;
        }

        /**
         * Returns true if the data transmit from the buffer is already started
         */
        bool transmitStarted() {
            return ready && readPos > 0;
        }

    private:
        volatile uint8_t buffer[BUFFER_SIZE];
        volatile uint8_t readPos;
        volatile uint8_t writePos;
        volatile bool ready; // true if the buffer data is ready to transmit
};

#endif