#include <signal.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <thread>
#include <unitree/common/thread/thread.hpp>

#include "param.h"
#include "reset_dds_wrapper.hpp"

class ResetPublisher
{
   public:
    ResetPublisher(const std::string& topic = "rt/mjc/reset", int interval_ms = 1000)
        : reset_pub_(std::make_shared<ResetPubs>(topic)), interval_ms_(interval_ms), running_(false)
    {
        // Initialize the message with current time
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

        reset_pub_->msg_.sec() = static_cast<int32_t>(seconds.count());
        reset_pub_->msg_.nanosec() = static_cast<uint32_t>(nanoseconds.count());
    }

    void start()
    {
        if (running_)
        {
            std::cout << "ResetPublisher is already running!" << std::endl;
            return;
        }

        running_ = true;
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "reset_publisher", UT_CPU_ID_NONE, interval_ms_ * 1000,
            std::bind(&ResetPublisher::publishReset, this));

        std::cout << "ResetPublisher started, publishing reset signals every " << interval_ms_
                  << "ms" << std::endl;
    }

    void stop()
    {
        if (!running_)
        {
            return;
        }

        running_ = false;
        if (thread_)
        {
            thread_.reset();
        }

        std::cout << "ResetPublisher stopped" << std::endl;
    }

    void setInterval(int interval_ms)
    {
        interval_ms_ = interval_ms;
        if (running_)
        {
            // Restart with new interval
            stop();
            start();
        }
    }

   private:
    void publishReset()
    {
        if (!running_)
        {
            return;
        }

        // Update timestamp
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

        reset_pub_->msg_.sec() = static_cast<int32_t>(seconds.count());
        reset_pub_->msg_.nanosec() = static_cast<uint32_t>(nanoseconds.count());

        // Publish the reset signal
        if (reset_pub_->trylock())
        {
            reset_pub_->unlockAndPublish();
            std::cout << "Reset signal published at " << reset_pub_->msg_.sec() << "s "
                      << reset_pub_->msg_.nanosec() << "ns" << std::endl;
        }
        else
        {
            std::cout << "Failed to acquire lock for reset signal publication" << std::endl;
        }
    }

    std::shared_ptr<ResetPubs> reset_pub_;
    std::shared_ptr<unitree::common::RecurrentThread> thread_;
    int interval_ms_;
    std::atomic<bool> running_;
};

// Global publisher for signal handling
std::unique_ptr<ResetPublisher> g_reset_publisher;

void signalHandler(int signal)
{
    std::cout << "\nReceived signal " << signal << ", stopping ResetPublisher..." << std::endl;
    if (g_reset_publisher)
    {
        g_reset_publisher->stop();
    }
    exit(0);
}

int main(int argc, char* argv[])
{
    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Load configuration from YAML file
    std::filesystem::path proj_dir = std::filesystem::current_path();
    std::string config_file = (proj_dir / "config.yaml").string();

    try
    {
        param::config.load_from_yaml(config_file);
        std::cout << "Loaded config: domain_id=" << param::config.domain_id
                  << ", interface=" << param::config.interface << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to load config.yaml: " << e.what() << std::endl;
        std::cerr << "Using default values: domain_id=1, interface=lo" << std::endl;
        param::config.domain_id = 1;
        param::config.interface = "lo";
    }

    // Initialize DDS system - this is crucial!
    std::cout << "Initializing DDS system..." << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(param::config.domain_id,
                                                     param::config.interface);
    std::cout << "DDS system initialized successfully" << std::endl;

    // Parse command line arguments
    int interval_ms = 1000;  // Default 1 second
    if (argc > 1)
    {
        try
        {
            interval_ms = std::stoi(argv[1]);
            if (interval_ms <= 0)
            {
                std::cerr << "Error: Interval must be positive" << std::endl;
                return 1;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error: Invalid interval value: " << argv[1] << std::endl;
            return 1;
        }
    }

    std::cout << "=== Unitree Reset Publisher Test ===" << std::endl;
    std::cout << "Usage: " << argv[0] << " [interval_ms]" << std::endl;
    std::cout << "Publishing reset signals every " << interval_ms << "ms" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << "=====================================" << std::endl;

    // Create and start the reset publisher
    g_reset_publisher = std::make_unique<ResetPublisher>("rt/mjc/reset", interval_ms);
    g_reset_publisher->start();

    // Keep the main thread alive
    try
    {
        while (true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    // Cleanup
    if (g_reset_publisher)
    {
        g_reset_publisher->stop();
    }

    return 0;
}
