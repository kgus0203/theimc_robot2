#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <thread>

#include "FRTcpClient.h"

TEST(FairinoSdkSocketShutdown, CloseUnblocksReceiveAndIsIdempotent)
{
  const int listener = socket(AF_INET, SOCK_STREAM, 0);
  ASSERT_GE(listener, 0);

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  ASSERT_EQ(bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0);
  ASSERT_EQ(listen(listener, 1), 0);

  socklen_t address_size = sizeof(address);
  ASSERT_EQ(getsockname(listener, reinterpret_cast<sockaddr*>(&address), &address_size), 0);

  std::thread server([listener]() {
    const int connection = accept(listener, nullptr, nullptr);
    if (connection >= 0) {
      std::array<char, 16> buffer{};
      recv(connection, buffer.data(), buffer.size(), 0);
      close(connection);
    }
    close(listener);
  });

  FRTcpClient client("127.0.0.1", ntohs(address.sin_port));
  ASSERT_EQ(client.Connect(), 0);
  client.SetReConnectParam(false, 0, 50);

  std::array<char, 4096> frame{};
  std::thread receiver([&client, &frame]() { client.RecvCNDEPkg(frame.data()); });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  EXPECT_EQ(client.Close(), 0);
  EXPECT_EQ(client.Close(), 0);
  receiver.join();
  server.join();
}
