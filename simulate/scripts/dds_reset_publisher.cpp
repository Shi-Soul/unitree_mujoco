#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

// Simple reset message structure for DDS (same as in main_dds.cc)
struct ResetMessage
{
    uint32_t timestamp;
    bool reset_flag;
};

class ResetPublisher
{
   public:
    ResetPublisher()
    {
        // Initialize DDS
        unitree::robot::ChannelFactory::Instance()->Init(0, "");

        // Create the publisher for reset commands
        reset_pub_ = std::make_unique<unitree::robot::ChannelPublisher<ResetMessage>>("mjc/reset");
        std::cout << "Reset publisher initialized. Publishing on 'mjc/reset' topic." << std::endl;
    }

    void sendResetCommand()
    {
        ResetMessage msg;
        msg.timestamp =
            static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count());
        msg.reset_flag = true;

        reset_pub_->Write(msg);
        std::cout << "Reset command sent at timestamp: " << msg.timestamp << std::endl;
    }

   private:
    std::unique_ptr<unitree::robot::ChannelPublisher<ResetMessage>> reset_pub_;
};

void printUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --once    Send a single reset command and exit\n";
    std::cout << "  --loop    Send reset commands every 10 seconds (default)\n";
    std::cout << "  --help    Show this help message\n";
}

int main(int argc, char* argv[])
{
    bool send_once = false;
    bool show_help = false;

    // Parse command line arguments
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--once")
        {
            send_once = true;
        }
        else if (arg == "--loop")
        {
            send_once = false;
        }
        else if (arg == "--help")
        {
            show_help = true;
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    if (show_help)
    {
        printUsage(argv[0]);
        return 0;
    }

    std::cout << "=== MuJoCo DDS Reset Publisher ===" << std::endl;

    try
    {
        ResetPublisher publisher;

        if (send_once)
        {
            std::cout << "Sending single reset command..." << std::endl;
            publisher.sendResetCommand();
            std::cout << "Reset command sent successfully." << std::endl;
        }
        else
        {
            std::cout << "Sending reset commands every 10 seconds. Press Ctrl+C to stop."
                      << std::endl;

            // Send initial reset command
            publisher.sendResetCommand();

            // Send reset commands every 10 seconds
            while (true)
            {
                std::this_thread::sleep_for(std::chrono::seconds(10));
                publisher.sendResetCommand();
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}