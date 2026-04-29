#include "uart_interface.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <errno.h>

UARTInterface::UARTInterface()
    : fileDescriptor_(-1)
    , initialized_(false)
    , baudRate_(115200)
    , bytesSent_(0)
    , bytesReceived_(0)
    , errors_(0)
    , dataCallback_(nullptr) {
}

UARTInterface::~UARTInterface() {
    close();
}

bool UARTInterface::initialize(const std::string& devicePath, int baudRate) {
    if (initialized_) {
        std::cerr << "UARTInterface already initialized" << std::endl;
        return false;
    }

    devicePath_ = devicePath;
    baudRate_ = baudRate;

    // Open the device
    fileDescriptor_ = ::open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fileDescriptor_ < 0) {
        std::cerr << "Failed to open UART device: " << devicePath << " - " << strerror(errno) << std::endl;
        return false;
    }

    // Configure the UART
    if (!setTermios()) {
        ::close(fileDescriptor_);
        fileDescriptor_ = -1;
        return false;
    }

    initialized_ = true;
    std::cout << "UART interface initialized on: " << devicePath << " at " << baudRate << " baud" << std::endl;
    return true;
}

bool UARTInterface::configure(int dataBits, int stopBits, int parity) {
    if (!initialized_) {
        std::cerr << "UARTInterface not initialized" << std::endl;
        return false;
    }

    struct termios options;
    if (tcgetattr(fileDescriptor_, &options) != 0) {
        std::cerr << "Failed to get UART attributes: " << strerror(errno) << std::endl;
        return false;
    }

    // Set data bits
    options.c_cflag &= ~CSIZE;
    switch (dataBits) {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
        default:
            options.c_cflag |= CS8;
            break;
    }

    // Set stop bits
    if (stopBits == 2) {
        options.c_cflag |= CSTOPB;
    } else {
        options.c_cflag &= ~CSTOPB;
    }

    // Set parity
    switch (parity) {
        case 1:  // Odd parity
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            break;
        case 2:  // Even parity
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            break;
        case 0:  // No parity
        default:
            options.c_cflag &= ~PARENB;
            break;
    }

    // Apply the configuration
    if (tcsetattr(fileDescriptor_, TCSANOW, &options) != 0) {
        std::cerr << "Failed to set UART attributes: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

int UARTInterface::sendData(const uint8_t* data, int size) {
    if (!initialized_ || !data || size <= 0) {
        return -1;
    }

    int bytesWritten = write(fileDescriptor_, data, size);
    if (bytesWritten < 0) {
        std::cerr << "Failed to write to UART: " << strerror(errno) << std::endl;
        errors_++;
        return -1;
    }

    // Ensure data is transmitted
    tcdrain(fileDescriptor_);

    bytesSent_ += bytesWritten;
    return bytesWritten;
}

int UARTInterface::sendString(const std::string& str) {
    return sendData(reinterpret_cast<const uint8_t*>(str.c_str()), str.length());
}

int UARTInterface::receiveData(uint8_t* buffer, int bufferSize, int timeoutMs) {
    if (!initialized_ || !buffer || bufferSize <= 0) {
        return -1;
    }

    // Set up timeout if specified
    if (timeoutMs >= 0) {
        struct termios options;
        if (tcgetattr(fileDescriptor_, &options) == 0) {
            // Convert timeout to deciseconds (VTIME is in 0.1s units)
            options.c_cc[VMIN] = 0;
            options.c_cc[VTIME] = timeoutMs / 100;
            tcsetattr(fileDescriptor_, TCSANOW, &options);
        }
    }

    int bytesRead = read(fileDescriptor_, buffer, bufferSize);
    if (bytesRead < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "Failed to read from UART: " << strerror(errno) << std::endl;
            errors_++;
            return -1;
        }
        return 0;  // No data available
    }

    bytesReceived_ += bytesRead;
    return bytesRead;
}

void UARTInterface::setDataCallback(DataCallback callback) {
    dataCallback_ = callback;
}

int UARTInterface::processData() {
    if (!initialized_ || !dataCallback_) {
        return 0;
    }

    int processed = 0;
    uint8_t buffer[1024];  // Buffer for received data

    while (true) {
        int size = receiveData(buffer, sizeof(buffer), 0);  // Non-blocking
        if (size <= 0) {
            break;  // No more data
        }

        dataCallback_(buffer, size);
        processed++;
    }

    return processed;
}

bool UARTInterface::isInitialized() const {
    return initialized_;
}

void UARTInterface::getStatistics(uint64_t& bytesSent, uint64_t& bytesReceived,
                                 uint64_t& errors) const {
    bytesSent = bytesSent_;
    bytesReceived = bytesReceived_;
    errors = errors_;
}

void UARTInterface::resetStatistics() {
    bytesSent_ = 0;
    bytesReceived_ = 0;
    errors_ = 0;
}

void UARTInterface::flushReceive() {
    if (initialized_) {
        tcflush(fileDescriptor_, TCIFLUSH);
    }
}

void UARTInterface::flushTransmit() {
    if (initialized_) {
        tcflush(fileDescriptor_, TCOFLUSH);
    }
}

void UARTInterface::close() {
    if (initialized_) {
        if (fileDescriptor_ >= 0) {
            ::close(fileDescriptor_);
            fileDescriptor_ = -1;
        }
        initialized_ = false;
    }
}

bool UARTInterface::setBaudRate(int baudRate) {
    struct termios options;
    if (tcgetattr(fileDescriptor_, &options) != 0) {
        return false;
    }

    speed_t speed = getBaudRateConstant(baudRate);
    if (speed == B0) {
        std::cerr << "Unsupported baud rate: " << baudRate << std::endl;
        return false;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    return tcsetattr(fileDescriptor_, TCSANOW, &options) == 0;
}

bool UARTInterface::setTermios() {
    struct termios options;

    // Get current options
    if (tcgetattr(fileDescriptor_, &options) != 0) {
        std::cerr << "Failed to get UART attributes: " << strerror(errno) << std::endl;
        return false;
    }

    // Set baud rate
    if (!setBaudRate(baudRate_)) {
        return false;
    }

    // Configure for 8N1 (8 data bits, no parity, 1 stop bit)
    options.c_cflag &= ~PARENB;   // No parity
    options.c_cflag &= ~CSTOPB;   // 1 stop bit
    options.c_cflag &= ~CSIZE;    // Clear data bit mask
    options.c_cflag |= CS8;      // 8 data bits

    // Enable receiver, ignore modem control lines
    options.c_cflag |= (CLOCAL | CREAD);

    // Raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Raw output mode
    options.c_oflag &= ~OPOST;

    // Disable software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Disable hardware flow control
    options.c_cflag &= ~CRTSCTS;

    // Set read timeouts
    options.c_cc[VMIN] = 0;   // Non-blocking read
    options.c_cc[VTIME] = 0;  // No timeout

    // Apply the configuration
    if (tcsetattr(fileDescriptor_, TCSANOW, &options) != 0) {
        std::cerr << "Failed to set UART attributes: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

int UARTInterface::getBaudRateConstant(int baudRate) {
    switch (baudRate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B0;  // Unsupported
    }
}