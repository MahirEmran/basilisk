#ifndef UART_INTERFACE_H
#define UART_INTERFACE_H

#include <string>
#include <cstdint>
#include <functional>

/**
 * @brief UART interface for serial communication
 *
 * This class provides a high-level interface for UART-based
 * serial communication on the BeagleBone Black.
 */
class UARTInterface {
public:
    // Callback types for data handling
    using DataCallback = std::function<void(const uint8_t*, int)>;

    /**
     * @brief Constructor
     */
    UARTInterface();

    /**
     * @brief Destructor
     */
    ~UARTInterface();

    /**
     * @brief Initialize UART interface
     * @param devicePath UART device path (e.g., "/dev/ttyO0", "/dev/ttyS0")
     * @param baudRate Baud rate (e.g., 9600, 115200)
     * @return true if initialization successful, false otherwise
     */
    bool initialize(const std::string& devicePath, int baudRate);

    /**
     * @brief Configure UART parameters
     * @param dataBits Number of data bits (5, 6, 7, or 8)
     * @param stopBits Number of stop bits (1 or 2)
     * @param parity Parity (0=none, 1=odd, 2=even)
     * @return true if configuration successful, false otherwise
     */
    bool configure(int dataBits, int stopBits, int parity);

    /**
     * @brief Send data over UART
     * @param data Data to send
     * @param size Number of bytes to send
     * @return Number of bytes sent, or -1 on error
     */
    int sendData(const uint8_t* data, int size);

    /**
     * @brief Send string over UART
     * @param str String to send
     * @return Number of bytes sent, or -1 on error
     */
    int sendString(const std::string& str);

    /**
     * @brief Receive data from UART (blocking)
     * @param buffer Output buffer for received data
     * @param bufferSize Size of output buffer
     * @param timeoutMs Timeout in milliseconds (0 for non-blocking, -1 for infinite)
     * @return Number of bytes received, or -1 on error/timeout
     */
    int receiveData(uint8_t* buffer, int bufferSize, int timeoutMs = -1);

    /**
     * @brief Set callback for received data
     * @param callback Function to call when data is received
     */
    void setDataCallback(DataCallback callback);

    /**
     * @brief Process incoming data (non-blocking)
     * @return Number of bytes processed
     */
    int processData();

    /**
     * @brief Check if interface is initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;

    /**
     * @brief Get statistics about the interface
     * @param bytesSent Output parameter for bytes sent
     * @param bytesReceived Output parameter for bytes received
     * @param errors Output parameter for error count
     */
    void getStatistics(uint64_t& bytesSent, uint64_t& bytesReceived,
                      uint64_t& errors) const;

    /**
     * @brief Reset statistics
     */
    void resetStatistics();

    /**
     * @brief Flush receive buffer
     */
    void flushReceive();

    /**
     * @brief Flush transmit buffer
     */
    void flushTransmit();

    /**
     * @brief Close the UART interface
     */
    void close();

private:
    int fileDescriptor_;      // File descriptor for UART device
    bool initialized_;       // Initialization status
    std::string devicePath_; // Device path
    int baudRate_;          // Baud rate

    // Statistics
    uint64_t bytesSent_;
    uint64_t bytesReceived_;
    uint64_t errors_;

    // Callback for data handling
    DataCallback dataCallback_;

    // Helper methods
    bool setBaudRate(int baudRate);
    bool setTermios();
    int getBaudRateConstant(int baudRate);
};

#endif // UART_INTERFACE_H