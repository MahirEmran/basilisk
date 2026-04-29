#include "bbb_gateway.h"
#include <iostream>
#include <string>
#include <csignal>
#include <unistd.h>
#include <atomic>

// Global flag for signal handling
static std::atomic<bool> keepRunning(true);
static BBBGateway* globalGateway = nullptr;

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    keepRunning = false;
    if (globalGateway) {
        globalGateway->stop();
    }
}

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --config <path>       Path to configuration file (default: config/gateway_config.yaml)" << std::endl;
    std::cout << "  --mode <mode>         Operation mode:" << std::endl;
    std::cout << "                          NETWORK_ONLY, UART_ONLY, SPI_ONLY," << std::endl;
    std::cout << "                          NETWORK_UART, NETWORK_SPI, ALL_INTERFACES" << std::endl;
    std::cout << "  --rate <hz>           Update rate in Hz (default: 100.0)" << std::endl;
    std::cout << "  --help                Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << programName << " --config /etc/bbb_gateway/config.yaml" << std::endl;
    std::cout << "  " << programName << " --mode NETWORK_ONLY --rate 50.0" << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "  BBB Gateway for Active Guidance" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // Default configuration
    std::string configPath = "config/gateway_config.yaml";
    std::string modeStr = "NETWORK_ONLY";
    double updateRate = 100.0;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--config") {
            if (i + 1 < argc) {
                configPath = argv[++i];
            } else {
                std::cerr << "Error: --config requires a path argument" << std::endl;
                return 1;
            }
        } else if (arg == "--mode") {
            if (i + 1 < argc) {
                modeStr = argv[++i];
            } else {
                std::cerr << "Error: --mode requires a mode argument" << std::endl;
                return 1;
            }
        } else if (arg == "--rate") {
            if (i + 1 < argc) {
                updateRate = std::stod(argv[++i]);
            } else {
                std::cerr << "Error: --rate requires a rate argument" << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Error: Unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    // Set up signal handler
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Create gateway
    BBBGateway gateway;
    globalGateway = &gateway;

    // Set operation mode
    if (modeStr == "NETWORK_ONLY") {
        gateway.setOperationMode(BBBGateway::MODE_NETWORK_ONLY);
    } else if (modeStr == "UART_ONLY") {
        gateway.setOperationMode(BBBGateway::MODE_UART_ONLY);
    } else if (modeStr == "SPI_ONLY") {
        gateway.setOperationMode(BBBGateway::MODE_SPI_ONLY);
    } else if (modeStr == "NETWORK_UART") {
        gateway.setOperationMode(BBBGateway::MODE_NETWORK_UART);
    } else if (modeStr == "NETWORK_SPI") {
        gateway.setOperationMode(BBBGateway::MODE_NETWORK_SPI);
    } else if (modeStr == "ALL_INTERFACES") {
        gateway.setOperationMode(BBBGateway::MODE_ALL_INTERFACES);
    } else {
        std::cerr << "Error: Invalid operation mode: " << modeStr << std::endl;
        return 1;
    }

    // Set update rate
    gateway.setUpdateRate(updateRate);

    // Initialize gateway
    if (!gateway.initialize(configPath)) {
        std::cerr << "Failed to initialize gateway" << std::endl;
        return 1;
    }

    // Start gateway
    if (!gateway.start()) {
        std::cerr << "Failed to start gateway" << std::endl;
        return 1;
    }

    std::cout << "Gateway is running. Press Ctrl+C to stop." << std::endl;
    std::cout << std::endl;

    // Main loop - print statistics periodically
    while (keepRunning && gateway.getStatus() == BBBGateway::STATUS_RUNNING) {
        sleep(5);  // Print statistics every 5 seconds

        uint64_t uptime, loopCount;
        uint32_t missedDeadlines;
        double avgLoopTime;

        gateway.getStatistics(uptime, loopCount, missedDeadlines, avgLoopTime);

        std::cout << "Statistics:" << std::endl;
        std::cout << "  Uptime: " << uptime << " seconds" << std::endl;
        std::cout << "  Loop count: " << loopCount << std::endl;
        std::cout << "  Missed deadlines: " << missedDeadlines << std::endl;
        std::cout << "  Average loop time: " << avgLoopTime << " microseconds" << std::endl;
        std::cout << std::endl;
    }

    // Stop gateway
    gateway.stop();

    std::cout << "Gateway shutdown complete" << std::endl;
    return 0;
}