#pragma once

#include <boost/asio/serial_port.hpp>
#include <GaiaByteUtility/GaiaByteUtility.hpp>
#include <future>
#include <atomic>

namespace Gaia::SerialIO
{
    /**
     * @brief The abstraction of real serial port device, providing read and write operations.
     * @details
     *  Options of this class can be stored and take effect immediately when the serial port is open.
     *  The serial port must be open before it's used for input or output.
     *  According to the mechanism of the serial port transportation, the open operation will fail
     *  when the serial port device does not exist, but it can never know whether this serial port device is properly
     *  connected to another serial port or not.
     */
    class SerialPort
    {
    public:
        /// Enum type of flow controlling.
        using FlowControlEnum = boost::asio::serial_port::flow_control;
        /// Enum type of parity checking.
        using ParityEnum = boost::asio::serial_port::parity;
        /// Enum type of stop bits.
        using StopBitsEnum = boost::asio::serial_port::stop_bits;

    private:
        //==============================
        // Settings
        //==============================

        /// Structure of detailed settings of a serial port device.
        struct{
            /**
             * @brief File name of the serial port device.
             * @detail
             *  This function will fail only if the file doesn't exist.
             *  According to the features of serial port devices, this function will succeed if the given file exists,
             *  no matter the connection is actually built or not.
             */
            std::string DeviceFileName;

            /**
             * @brief Setting option of the baud rate.
             */
            unsigned int BaudRate;

            /**
             * @brief Setting option of the size of a character.
             */
            unsigned int CharacterSize;

            /**
             * @brief Setting option of the method the device used to control the flow.
             */
            FlowControlEnum::type FlowControl;
            /**
             * @brief The parity method to use.
             */
            ParityEnum::type Parity;
            /**
             * @brief The stop bits to use.
             */
            StopBitsEnum::type StopBits;
        }DeviceSettings;

    protected:
        //==============================
        // IO Objects
        //==============================

        /// Execution context for IO operations.
        boost::asio::io_context IOContext {};
        /// Object for serial IO operations.
        boost::asio::serial_port IODevice {IOContext};

        std::future<void> ListenerBlocker;
        std::atomic_bool  ListenerLifeFlag {true};

        /// Continue to listen for incoming data.
        void ResumeListen();
        /// Pause the listening task so the IODevice can be used for read/write task.
        void PauseListen();
        /// Process the data received in the listening task.
        void ReceiveData(std::shared_ptr<std::vector<unsigned char>> buffer);

    public:
        //==============================
        // Constructors and Destructors
        //==============================

        /**
         * @brief Construct and bind to a device automatically.
         * @param file_name File name of the serial port file.
         * @attention This constructor will not open the serial port file automatically.
         * @details
         *  Attention, this function will only store the basic options of the serial port, but will not open
         *  the serial port device automatically.
         */
        explicit SerialPort(std::string file_name,
                unsigned int baud_rate = 115200, unsigned int character_size = 8,
                FlowControlEnum::type flow_control = FlowControlEnum::none,
                ParityEnum::type parity = ParityEnum::none,
                StopBitsEnum::type stop_bits = StopBitsEnum::one
        );

        /// Destruct and close the device if it's still open.
        virtual ~SerialPort();

    public:
        /// Size of the default buffer for incoming bytes.
        unsigned long DataBufferSize {256};

        /// Triggered when received data.
        std::function<void(std::shared_ptr<std::vector<unsigned char>>)> OnReceivedData;

        //==============================
        // Basic Control
        //==============================

        /**
         * @brief Open the serial port device.
         * @details This function will open the device using the options given in the constructor.
         */
        virtual void Open();

        /**
         * @brief Open the given serial port device.
         * @param file_path The path of the mapping device file.
         */
        virtual void Open(std::string file_path);

        /**
         * @brief Close the serial port device.
         */
        virtual void Close();

        /**
         * @brief Query whether this device is open or not.
         * @retval true This device is open.
         * @retval false This device is not open.
         */
        [[nodiscard]] inline bool IsOpen()
        {
            return IODevice.is_open();
        }

        //==============================
        // Options of the Serial Port
        //==============================

        /**
         * @brief Set the baud rate of the serial port.
         * @param baud_rate
         * @details This function will take effect immediately.
         */
        virtual void SetBaudRate(unsigned int baud_rate);
        /**
         * @brief Set the count of bits of a character.
         * @param size The count of bits of a character.
         * @details This function will take effect immediately.
         */
        virtual void SetCharacterSize(unsigned int size);
        /**
         * @brief Set the method the hardware using to control the flow.
         * @param flow_control Flow control method to use.
         * @details This function will take effect immediately.
         */
        virtual void SetFlowControlType(FlowControlEnum::type flow_control);
        /**
         * @brief Set the method the hardware using to perform the parity check.
         * @param parity Parity check method to use.
         * @details This function will take effect immediately.
         */
        virtual void SetParityType(ParityEnum::type parity);
        /**
         * @brief Set the type of the stop bits of a character.
         * @param stop_bits The type of stop bits of a character.
         * @details This function will take effect immediately.
         */
        virtual void SetStopBitsType(StopBitsEnum::type stop_bits);

        //==============================
        // IO Functions
        //==============================

        /**
         * @brief Write data into the port.
         * @param address Address of bytes to write.
         * @pre The serial port is open.
         */
        void Write(ByteUtility::BytesAddress address);

        /**
         * @brief Read uncertain amount of bytes.
         * @return Buffer of the read bytes.
         * @details It will use DataBufferSize as the default size of the buffer.
         * @pre The serial port is open.
         */
        ByteUtility::BytesBuffer Read();

        /**
         * @brief Read the given amount of bytes.
         * @return Buffer of the read bytes of the given length.
         * @details This function will not return until it read the given amount of bytes.
         * @pre The serial port is open.
         */
        ByteUtility::BytesBuffer Read(std::size_t size);

        //==============================
        // Overloaded Operators
        //==============================

        /// Stream operator used for write bytes.
        SerialPort& operator<<(ByteUtility::BytesAddress data);

        /// Stream operator used for read bytes.
        SerialPort& operator>>(ByteUtility::BytesBuffer& data);
    };
}