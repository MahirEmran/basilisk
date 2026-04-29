#include "spi_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>
#include <errno.h>

SPIInterface::SPIInterface()
    : fileDescriptor_(-1)
    , initialized_(false)
    , speedHz_(1000000)
    , mode_(MODE_0)
    , bitOrder_(MSB_FIRST)
    , bitsPerWord_(8)
    , bytesTransferred_(0)
    , transfers_(0)
    , errors_(0)
    , dataCallback_(nullptr) {
}

SPIInterface::~SPIInterface() {
    close();
}

bool SPIInterface::initialize(const std::string& devicePath, uint32_t speedHz) {
    if (initialized_) {
        std::cerr << "SPIInterface already initialized" << std::endl;
        return false;
    }

    devicePath_ = devicePath;
    speedHz_ = speedHz;

    // Open the device
    fileDescriptor_ = ::open(devicePath.c_str(), O_RDWR);
    if (fileDescriptor_ < 0) {
        std::cerr << "Failed to open SPI device: " << devicePath << " - " << strerror(errno) << std::endl;
        return false;
    }

    // Set default configuration
    if (!setSPIConfig()) {
        ::close(fileDescriptor_);
        fileDescriptor_ = -1;
        return false;
    }

    initialized_ = true;
    std::cout << "SPI interface initialized on: " << devicePath << " at " << speedHz << " Hz" << std::endl;
    return true;
}

bool SPIInterface::configure(SPIMode mode, SPIBitOrder bitOrder, uint8_t bitsPerWord) {
    if (!initialized_) {
        std::cerr << "SPIInterface not initialized" << std::endl;
        return false;
    }

    mode_ = mode;
    bitOrder_ = bitOrder;
    bitsPerWord_ = bitsPerWord;

    return setSPIConfig();
}

int SPIInterface::transfer(const uint8_t* txData, uint8_t* rxData, int size) {
    if (!initialized_ || !txData || !rxData || size <= 0) {
        return -1;
    }

    struct spi_ioc_transfer transfer;
    memset(&transfer, 0, sizeof(transfer));

    transfer.tx_buf = reinterpret_cast<unsigned long>(txData);
    transfer.rx_buf = reinterpret_cast<unsigned long>(rxData);
    transfer.len = size;
    transfer.speed_hz = speedHz_;
    transfer.bits_per_word = bitsPerWord_;
    transfer.cs_change = 0;  // Keep CS active between transfers
    transfer.delay_usecs = 0;

    int result = ioctl(fileDescriptor_, SPI_IOC_MESSAGE(1), &transfer);
    if (result < 0) {
        std::cerr << "Failed to perform SPI transfer: " << strerror(errno) << std::endl;
        errors_++;
        return -1;
    }

    bytesTransferred_ += size;
    transfers_++;
    return size;
}

int SPIInterface::write(const uint8_t* data, int size) {
    if (!initialized_ || !data || size <= 0) {
        return -1;
    }

    // For write-only, we can use a dummy receive buffer
    std::vector<uint8_t> dummyBuffer(size);
    return transfer(data, dummyBuffer.data(), size);
}

int SPIInterface::read(uint8_t* buffer, int size) {
    if (!initialized_ || !buffer || size <= 0) {
        return -1;
    }

    // For read-only, we can use a dummy transmit buffer
    std::vector<uint8_t> dummyBuffer(size, 0xFF);  // Send 0xFF for clock generation
    return transfer(dummyBuffer.data(), buffer, size);
}

int SPIInterface::writeRead(const uint8_t* txData, uint8_t* rxData, int size) {
    return transfer(txData, rxData, size);
}

bool SPIInterface::multiTransfer(const std::vector<std::vector<uint8_t>>& txData,
                               std::vector<std::vector<uint8_t>>& rxData) {
    if (!initialized_) {
        return false;
    }

    size_t numTransfers = txData.size();
    if (numTransfers == 0) {
        return true;
    }

    // Prepare receive buffers
    rxData.resize(numTransfers);
    for (size_t i = 0; i < numTransfers; ++i) {
        rxData[i].resize(txData[i].size());
    }

    // Create transfer structures
    std::vector<struct spi_ioc_transfer> transfers(numTransfers);
    for (size_t i = 0; i < numTransfers; ++i) {
        memset(&transfers[i], 0, sizeof(transfers[i]));
        transfers[i].tx_buf = reinterpret_cast<unsigned long>(txData[i].data());
        transfers[i].rx_buf = reinterpret_cast<unsigned long>(rxData[i].data());
        transfers[i].len = txData[i].size();
        transfers[i].speed_hz = speedHz_;
        transfers[i].bits_per_word = bitsPerWord_;
        transfers[i].cs_change = (i < numTransfers - 1) ? 1 : 0;  // Keep CS active except for last transfer
        transfers[i].delay_usecs = 0;
    }

    // Perform all transfers in one ioctl call
    int result = ioctl(fileDescriptor_, SPI_IOC_MESSAGE(numTransfers), transfers.data());
    if (result < 0) {
        std::cerr << "Failed to perform multi SPI transfer: " << strerror(errno) << std::endl;
        errors_++;
        return false;
    }

    // Update statistics
    for (const auto& tx : txData) {
        bytesTransferred_ += tx.size();
    }
    transfers_ += numTransfers;

    return true;
}

void SPIInterface::setDataCallback(DataCallback callback) {
    dataCallback_ = callback;
}

bool SPIInterface::isInitialized() const {
    return initialized_;
}

uint32_t SPIInterface::getSpeed() const {
    return speedHz_;
}

bool SPIInterface::setSpeed(uint32_t speedHz) {
    if (!initialized_) {
        return false;
    }

    speedHz_ = speedHz;
    return setSPIConfig();
}

void SPIInterface::getStatistics(uint64_t& bytesTransferred, uint64_t& transfers,
                                 uint64_t& errors) const {
    bytesTransferred = bytesTransferred_;
    transfers = transfers_;
    errors = errors_;
}

void SPIInterface::resetStatistics() {
    bytesTransferred_ = 0;
    transfers_ = 0;
    errors_ = 0;
}

void SPIInterface::close() {
    if (initialized_) {
        if (fileDescriptor_ >= 0) {
            ::close(fileDescriptor_);
            fileDescriptor_ = -1;
        }
        initialized_ = false;
    }
}

bool SPIInterface::setSPIConfig() {
    uint8_t mode = getSPIModeValue(mode_);
    uint8_t lsbFirst = (bitOrder_ == LSB_FIRST) ? 1 : 0;

    // Set SPI mode
    if (ioctl(fileDescriptor_, SPI_IOC_WR_MODE, &mode) < 0) {
        std::cerr << "Failed to set SPI mode: " << strerror(errno) << std::endl;
        return false;
    }

    // Set bit order
    if (ioctl(fileDescriptor_, SPI_IOC_WR_LSB_FIRST, &lsbFirst) < 0) {
        std::cerr << "Failed to set SPI bit order: " << strerror(errno) << std::endl;
        return false;
    }

    // Set bits per word
    if (ioctl(fileDescriptor_, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord_) < 0) {
        std::cerr << "Failed to set SPI bits per word: " << strerror(errno) << std::endl;
        return false;
    }

    // Set speed
    if (ioctl(fileDescriptor_, SPI_IOC_WR_MAX_SPEED_HZ, &speedHz_) < 0) {
        std::cerr << "Failed to set SPI speed: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

uint8_t SPIInterface::getSPIModeValue(SPIMode mode) {
    switch (mode) {
        case MODE_0: return SPI_MODE_0;  // CPOL=0, CPHA=0
        case MODE_1: return SPI_MODE_1;  // CPOL=0, CPHA=1
        case MODE_2: return SPI_MODE_2;  // CPOL=1, CPHA=0
        case MODE_3: return SPI_MODE_3;  // CPOL=1, CPHA=1
        default:    return SPI_MODE_0;
    }
}