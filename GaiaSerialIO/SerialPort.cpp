#include "SerialPort.hpp"

#include <utility>
#include <GaiaExceptions/GaiaExceptions.hpp>
#include <boost/asio.hpp>

namespace Gaia::SerialIO
{
    /// Constructor
    SerialPort::SerialPort(std::string file_name, unsigned int baud_rate, unsigned int character_size,
                           boost::asio::serial_port_base::flow_control::type flow_control,
                           boost::asio::serial_port_base::parity::type parity,
                           boost::asio::serial_port_base::stop_bits::type stop_bits) :
            DeviceSettings{.DeviceFileName = std::move(file_name), .BaudRate = baud_rate, .CharacterSize = character_size,
                           .FlowControl = flow_control, .Parity = parity, .StopBits = stop_bits}
    {
        ListenerBlocker = std::async(std::launch::async, [this, context = &IOContext, life_flag = &ListenerLifeFlag]() {
            while (life_flag->load())
            {
                context->run();
            }
        });
    }

    /// Destructor
    SerialPort::~SerialPort()
    {
        if (IsOpen())
        {
            IODevice.close();
        }
        ListenerLifeFlag = false;
        /// Wait for listener thread to stop.
        ListenerBlocker.get();
    }

    void SerialPort::ReceiveData(std::shared_ptr<std::vector<unsigned char>> buffer)
    {
        if (OnReceivedData)
        {
            OnReceivedData(std::move(buffer));
        }
        ResumeListen();
    }

    void SerialPort::ResumeListen()
    {
        auto buffer = std::make_shared<std::vector<unsigned char>>();
        buffer->resize(DataBufferSize);
        IODevice.async_read_some(boost::asio::buffer(buffer->data(), buffer->size()),[this, passed_buffer = buffer](const boost::system::error_code& error, std::size_t length)
        {
            if (error) return;
            passed_buffer->resize(length);
            this->ReceiveData(passed_buffer);
        });
    }

    void SerialPort::PauseListen()
    {
        IODevice.cancel();
    }

    /// Open the serial port device.
    void SerialPort::Open()
    {
        if (IODevice.is_open())
        {
            IODevice.close();
        }
        IODevice.open(DeviceSettings.DeviceFileName);

        IODevice.set_option(boost::asio::serial_port_base::baud_rate(DeviceSettings.BaudRate));
        IODevice.set_option(boost::asio::serial_port_base::character_size(DeviceSettings.CharacterSize));
        IODevice.set_option(boost::asio::serial_port_base::flow_control(DeviceSettings.FlowControl));
        IODevice.set_option(boost::asio::serial_port_base::parity(DeviceSettings.Parity));
        IODevice.set_option(boost::asio::serial_port_base::stop_bits(DeviceSettings.StopBits));
    }

    /// Open the serial port device.
    void SerialPort::Open(std::string file_path)
    {
        DeviceSettings.DeviceFileName = std::move(file_path);
        Open();
    }

    /// Close the serial port device.
    void SerialPort::Close()
    {
        if (IsOpen())
        {
            IODevice.cancel();
            IODevice.close();
        }
    }

    /// Set the baud rate of the serial port.
    void SerialPort::SetBaudRate(unsigned int baud_rate)
    {
        DeviceSettings.BaudRate = baud_rate;
        if (IODevice.is_open())
        {
            IODevice.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        }
    }

    /// Set the count of bits of a character.
    void SerialPort::SetCharacterSize(unsigned int size)
    {
        DeviceSettings.CharacterSize = size;
        if (IODevice.is_open())
        {
            IODevice.set_option(boost::asio::serial_port_base::character_size(size));
        }
    }

    /// Set the method the hardware using to control the flow.
    void SerialPort::SetFlowControlType(boost::asio::serial_port_base::flow_control::type flow_control)
    {
        DeviceSettings.FlowControl = flow_control;
        if (IODevice.is_open())
        {
            IODevice.set_option(boost::asio::serial_port_base::flow_control(flow_control));
        }
    }

    /// Set the method the hardware using to perform the parity check.
    void SerialPort::SetParityType(boost::asio::serial_port_base::parity::type parity)
    {
        DeviceSettings.Parity = parity;
        if (IODevice.is_open())
        {
            IODevice.set_option(boost::asio::serial_port_base::parity(parity));
        }
    }

    /// Set the type of the stop bits of a character.
    void SerialPort::SetStopBitsType(boost::asio::serial_port_base::stop_bits::type stop_bits)
    {
        DeviceSettings.StopBits = stop_bits;
        if (IODevice.is_open())
        {
            IODevice.set_option(boost::asio::serial_port_base::stop_bits(stop_bits));
        }
    }

    void SerialPort::Write(ByteUtility::BytesAddress address)
    {
        if (IODevice.is_open())
        {
            PauseListen();
            boost::asio::write(IODevice, boost::asio::buffer(address.Data, address.Length));
            ResumeListen();
        }
        else
        {
            throw Exceptions::ExceptionBase("SerialPort", "Device is not opened.");
        }
    }

    /// Read uncertain amount of bytes.
    ByteUtility::BytesBuffer SerialPort::Read()
    {
        if (IODevice.is_open())
        {
            ByteUtility::BytesBuffer buffer;
            buffer.resize(DataBufferSize);
            PauseListen();
            std::size_t real_length = IODevice.read_some(boost::asio::buffer(buffer));
            ResumeListen();
            buffer.resize(real_length);
            return buffer;
        }
        else
        {
            throw Exceptions::ExceptionBase("SerialPort", "Device is not opened.");
        }
    }

    /// Read the given amount of bytes.
    ByteUtility::BytesBuffer SerialPort::Read(std::size_t size)
    {
        if (IODevice.is_open())
        {
            ByteUtility::BytesBuffer buffer;
            buffer.resize(size);
            PauseListen();
            boost::asio::read(IODevice, boost::asio::buffer(buffer));
            ResumeListen();
            return buffer;
        }
        else
        {
            throw Exceptions::ExceptionBase("SerialPort", "Device is not opened.");
        }
    }

    /// Stream operator used for write bytes.
    SerialPort &SerialPort::operator<<(ByteUtility::BytesAddress data)
    {
        Write(data);
        return *this;
    }

    /// Stream operator used for read bytes.
    SerialPort &SerialPort::operator>>(ByteUtility::BytesBuffer &data)
    {
        data = Read();
        return *this;
    }
}