#ifndef SPI_INTERFACE_H
#define SPI_INTERFACE_H

#include <string>
#include <cstdint>
#include <vector>
#include <functional>

/**
 * @brief SPI interface for SPI communication
 *
 * This class provides a high-level interface for SPI-based
 * communication on the BeagleBone Black.
 */
class SPIInterface {
public:
    // SPI mode definitions
    enum SPIMode {
        MODE_0 = 0,  // CPOL=0, CPHA=0
        MODE_1 = 1,  // CPOL=0, CPHA=1
        MODE_2 = 2,  // CPOL=1, CPHA=0
        MODE_3 = 3   // CPOL=1, CPHA=1
    };

    // SPI bit order
    enum SPIBitOrder {
        MSB_FIRST = 0,
        LSB_FIRST = 1
    };

    // Callback types for data handling
    using DataCallback = std::function<void(const uint8_t*, int)>;

    /**
     * @brief Constructor
     */
    SPIInterface();

    /**
     * @brief Destructor
     */
    ~SPIInterface();

    /**
     * @brief Initialize SPI interface
     * @param devicePath SPI device path (e.g., "/dev/spidev0.0", "/dev/spidev1.0")
     * @param speedHz SPI clock speed in Hz
     * @return true if initialization successful, false otherwise
     */
    bool initialize(const std::string& devicePath, uint32_t speedHz);

    /**
     * @brief Configure SPI parameters
     * @param mode SPI mode (0-3)
     * @param bitOrder Bit order (MSB_FIRST or LSB_FIRST)
     * @param bitsPerWord Number of bits per word (usually 8)
     * @return true if configuration successful, false otherwise
     */
    bool configure(SPIMode mode, SPIBitOrder bitOrder, uint8_t bitsPerWord);

    /**
     * @brief Transfer data over SPI (full duplex)
     * @param txData Data to transmit
     * @param rxData Buffer for received data (can be same as txData for in-place)
     * @param size Number of bytes to transfer
     * @return Number of bytes transferred, or -1 on error
     */
    int transfer(const uint8_t* txData, uint8_t* rxData, int size);

    /**
     * @brief Write data over SPI (half duplex)
     * @param data Data to write
     * @param size Number of bytes to write
     * @return Number of bytes written, or -1 on error
     */
    int write(const uint8_t* data, int size);

    /**
     * @brief Read data over SPI (half duplex)
     * @param buffer Buffer for received data
     * @param size Number of bytes to read
     * @return Number of bytes read, or -1 on error
     */
    int read(uint8_t* buffer, int size);

    /**
     * @brief Write and read data in a single transaction
     * @param txData Data to transmit
     * @param rxData Buffer for received data
     * @param size Number of bytes to transfer
     * @return Number of bytes transferred, or -1 on error
     */
    int writeRead(const uint8_t* txData, uint8_t* rxData, int size);

    /**
     * @brief Perform multiple SPI transfers in a single transaction
     * @param transfers Vector of transfer descriptors
     * @return true if all transfers successful, false otherwise
     */
    bool multiTransfer(const std::vector<std::vector<uint8_t>>& txData,
                      std::vector<std::vector<uint8_t>>& rxData);

    /**
     * @brief Set callback for received data
     * @param callback Function to call when data is received
     */
    void setDataCallback(DataCallback callback);

    /**
     * @brief Check if interface is initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;

    /**
     * @brief Get current SPI speed
     * @return Current SPI speed in Hz
     */
    uint32_t getSpeed() const;

    /**
     * @brief Set SPI speed
     * @param speedHz New SPI speed in Hz
     * @return true if speed set successfully, false otherwise
     */
    bool setSpeed(uint32_t speedHz);

    /**
     * @brief Get statistics about the interface
     * @param bytesTransferred Output parameter for bytes transferred
     * @param transfers Output parameter for number of transfers
     * @param errors Output parameter for error count
     */
    void getStatistics(uint64_t& bytesTransferred, uint64_t& transfers,
                      uint64_t& errors) const;

    /**
     * @brief Reset statistics
     */
    void resetStatistics();

    /**
     * @brief Close the SPI interface
     */
    void close();

private:
    int fileDescriptor_;      // File descriptor for SPI device
    bool initialized_;       // Initialization status
    std::string devicePath_; // Device path
    uint32_t speedHz_;       // SPI clock speed

    // SPI configuration
    SPIMode mode_;
    SPIBitOrder bitOrder_;
    uint8_t bitsPerWord_;

    // Statistics
    uint64_t bytesTransferred_;
    uint64_t transfers_;
    uint64_t errors_;

    // Callback for data handling
    DataCallback dataCallback_;

    // Helper methods
    bool setSPIConfig();
    uint8_t getSPIModeValue(SPIMode mode);
};

#endif // SPI_INTERFACE_H