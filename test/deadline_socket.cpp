#include <BoostHelper.hpp>

#include <boost/asio.hpp>
#include <chrono>
#include <iostream>

int main() {
    // A quiet receiver may resume sending after the read deadline expires.
    namespace asio = boost::asio;
    using tcp = asio::ip::tcp;
    asio::io_context io;
    tcp::acceptor listener(io, tcp::endpoint(asio::ip::address_v4::loopback(), 0));
    DeadlineSocket<tcp::socket> receiver(io);
    receiver.socket().connect(listener.local_endpoint());
    tcp::socket sender(io);
    listener.accept(sender);

    char byte = 0;
    for (int attempt = 0; attempt < 3; ++attempt) {
        try {
            receiver.read_some(asio::buffer(&byte, 1), std::chrono::milliseconds(50));
            std::cerr << "Idle read unexpectedly completed\n";
            return 1;
        } catch (const boost::system::system_error &error) {
            if (error.code() != asio::error::timed_out) {
                std::cerr << "Idle read was reported as " << error.code().message()
                          << " instead of a timeout\n";
                return 1;
            }
        }
    }

    if (!receiver.socket().is_open()) {
        std::cerr << "Read timeout closed the receiver\n";
        return 1;
    }
    asio::write(sender, asio::buffer("X", 1));
    try {
        const auto count = receiver.read_some(asio::buffer(&byte, 1),
                                              std::chrono::milliseconds(500));
        if (count != 1 || byte != 'X') {
            std::cerr << "Receiver did not recover after idle timeout\n";
            return 1;
        }
    } catch (const boost::system::system_error &error) {
        std::cerr << "Read after timeout failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
