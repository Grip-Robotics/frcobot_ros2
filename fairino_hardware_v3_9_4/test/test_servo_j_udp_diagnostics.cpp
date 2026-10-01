#include "fairino_hardware/servo_j_udp_diagnostics.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <thread>

namespace
{
using fairino_hardware::ServoUdpCall;
using fairino_hardware::ServoUdpDiagnostics;
using fairino_hardware::ServoUdpReply;
using fairino_hardware::ServoUdpTracker;

ServoUdpReply reply(int64_t time, int counter, int command = 376)
{
  ServoUdpReply event{};
  event.mono_ns = time;
  event.counter = counter;
  event.frame_command = command;
  event.content_length = 1;
  event.actual_length = 1;
  event.retained_length = 1;
  event.content[0] = '1';
  return event;
}

void start(ServoUdpTracker & tracker, int counter = 0)
{
  tracker.call(ServoUdpCall{100, 689, -1});
  tracker.reply(reply(110, counter, 689));
}

TEST(ServoUdpDiagnostics, CorrelatesCounterWrapWithoutCallingItPhysicalCompletion)
{
  auto tracker = std::make_unique<ServoUdpTracker>();
  start(*tracker, 254);
  for (int index = 1; index <= 520; ++index) {
    tracker->call(ServoUdpCall{1000 + index * 10, 376, 5000 + index});
    tracker->reply(reply(1005 + index * 10, (254 + index) % 256));
  }
  EXPECT_EQ(tracker->snapshot().matched_replies, 520U);
  EXPECT_EQ(tracker->snapshot().last_replied_command_id, 5520);
  EXPECT_EQ(tracker->snapshot().last_reply_rtt_ns, 5);
  EXPECT_EQ(tracker->snapshot().start_reply_ns - tracker->snapshot().start_call_ns, 10);
  std::ostringstream out;
  tracker->write_summary(out);
  EXPECT_NE(out.str().find("\"physical_completion_known\":false"), std::string::npos);
}

TEST(ServoUdpDiagnostics, LostReplyDoesNotBecomeAFalseWrapMatch)
{
  auto tracker = std::make_unique<ServoUdpTracker>();
  start(*tracker);
  for (int index = 1; index <= 257; ++index) {
    tracker->call(ServoUdpCall{1000 + index * 10, 376, index});
    if (index != 1) {tracker->reply(reply(1005 + index * 10, index % 256));}
  }
  EXPECT_EQ(tracker->snapshot().matched_replies, 255U);
  EXPECT_EQ(tracker->snapshot().ambiguous, 1U);
  EXPECT_EQ(tracker->snapshot().last_replied_ordinal, 256U);
}

TEST(ServoUdpDiagnostics, RecordsOutOfOrderAndDuplicateReplies)
{
  auto tracker = std::make_unique<ServoUdpTracker>();
  start(*tracker);
  tracker->call(ServoUdpCall{200, 376, 21});
  tracker->call(ServoUdpCall{300, 376, 22});
  tracker->reply(reply(400, 2));
  tracker->reply(reply(500, 1));
  tracker->reply(reply(600, 2));
  EXPECT_EQ(tracker->snapshot().out_of_order, 1U);
  EXPECT_EQ(tracker->snapshot().unmatched, 1U);
  EXPECT_EQ(tracker->snapshot().last_replied_ordinal, 2U);
  EXPECT_EQ(tracker->snapshot().last_replied_command_id, 22);
}

TEST(ServoUdpDiagnostics, NewSessionClearsOldCandidatesAndRejectsEarlierReceiveTime)
{
  auto tracker = std::make_unique<ServoUdpTracker>();
  start(*tracker, 4);
  tracker->call(ServoUdpCall{200, 376, 21});
  tracker->call(ServoUdpCall{500, 689, -1});
  tracker->reply(reply(300, 5));
  tracker->reply(reply(510, 20, 689));
  tracker->call(ServoUdpCall{520, 376, 31});
  tracker->reply(reply(530, 5));
  tracker->reply(reply(540, 21));
  EXPECT_EQ(tracker->snapshot().session, 2U);
  EXPECT_EQ(tracker->snapshot().previous_unreplied, 1U);
  EXPECT_EQ(tracker->snapshot().stale, 1U);
  EXPECT_EQ(tracker->snapshot().unmatched, 1U);
  EXPECT_EQ(tracker->snapshot().matched_replies, 1U);
  EXPECT_EQ(tracker->snapshot().last_replied_command_id, 31);
}

TEST(ServoUdpDiagnostics, MissingPriorStartPreventsGuessingANewStartAnchor)
{
  auto tracker = std::make_unique<ServoUdpTracker>();
  tracker->call(ServoUdpCall{100, 689, -1});
  tracker->call(ServoUdpCall{500, 689, -1});
  tracker->reply(reply(510, 40, 689));
  EXPECT_EQ(tracker->snapshot().start_counter, -1);
  EXPECT_EQ(tracker->snapshot().ambiguous, 1U);
}

TEST(ServoUdpDiagnostics, FailedSdkSendInvalidatesCounterPrediction)
{
  auto tracker = std::make_unique<ServoUdpTracker>();
  start(*tracker);
  tracker->call(ServoUdpCall{200, 376, 21});
  tracker->call(ServoUdpCall{210, 376, -1, -15, true});
  tracker->reply(reply(220, 1));
  EXPECT_TRUE(tracker->snapshot().degraded);
  EXPECT_EQ(tracker->snapshot().matched_replies, 0U);
}

TEST(ServoUdpDiagnostics, StartAndEndRepliesHaveIndependentLatency)
{
  auto tracker = std::make_unique<ServoUdpTracker>();
  start(*tracker, 254);
  tracker->call(ServoUdpCall{200, 376, 21});
  tracker->reply(reply(230, 255));
  tracker->call(ServoUdpCall{300, 690, -1});
  tracker->reply(reply(340, 0, 690));
  EXPECT_EQ(tracker->snapshot().end_reply_ns, 340);
  EXPECT_EQ(tracker->snapshot().end_reply_ns - tracker->snapshot().end_call_ns, 40);
}

TEST(ServoUdpDiagnostics, MalformedReplyDoesNotConsumePendingCommand)
{
  auto tracker = std::make_unique<ServoUdpTracker>();
  start(*tracker);
  tracker->call(ServoUdpCall{200, 376, 21});
  auto event = reply(300, 1);
  event.content_length = 100;
  tracker->reply(event);
  tracker->reply(reply(400, 1));
  EXPECT_EQ(tracker->snapshot().malformed, 1U);
  EXPECT_EQ(tracker->snapshot().matched_replies, 1U);
}

TEST(ServoUdpDiagnostics, QueueFullDropsDiagnosticsWithoutOverwritingUnreadEntries)
{
  fairino_hardware::ServoDiagnosticQueue<int, 4> queue;
  EXPECT_TRUE(queue.push(10));
  EXPECT_TRUE(queue.push(20));
  EXPECT_TRUE(queue.push(30));
  EXPECT_FALSE(queue.push(40));
  EXPECT_EQ(queue.dropped(), 1U);
  int result = 0;
  EXPECT_TRUE(queue.pop(result));
  EXPECT_EQ(result, 10);
  EXPECT_TRUE(queue.push(50));
  EXPECT_TRUE(queue.pop(result));
  EXPECT_EQ(result, 20);
  EXPECT_TRUE(queue.pop(result));
  EXPECT_EQ(result, 30);
  EXPECT_TRUE(queue.pop(result));
  EXPECT_EQ(result, 50);
  EXPECT_FALSE(queue.pop(result));
}

TEST(ServoUdpDiagnostics, ShutdownIgnoresLateCallbackAndCannotReactivateOldMailbox)
{
  auto diagnostics = std::make_unique<ServoUdpDiagnostics>();
  ASSERT_TRUE(diagnostics->enable_once());
  diagnostics->call(689, -1, 100);
  diagnostics->receive(1, 7, 689, 1, "1", 110);
  std::ostringstream initial;
  diagnostics->write_summary(initial);
  diagnostics->disable();
  diagnostics->receive(1, 8, 376, 1, "1", 120);
  diagnostics->call(376, 22, 130);
  EXPECT_FALSE(diagnostics->enable_once());
  std::ostringstream history;
  diagnostics->write_history(history);
  const auto text = history.str();
  EXPECT_EQ(std::count(text.begin(), text.end(), '\n'), 1);
}

TEST(ServoUdpDiagnostics, DrainOrdersTwoShortSessionsWithoutInventingAMissingStart)
{
  auto diagnostics = std::make_unique<ServoUdpDiagnostics>();
  ASSERT_TRUE(diagnostics->enable_once());
  diagnostics->call(689, -1, 100);
  diagnostics->receive(1, 7, 689, 1, "1", 110);
  diagnostics->call(376, 81, 120);
  diagnostics->receive(1, 8, 376, 1, "1", 130);
  diagnostics->call(690, -1, 140);
  diagnostics->receive(1, 9, 690, 1, "1", 150);
  diagnostics->call(689, -1, 200);
  diagnostics->receive(1, 10, 689, 1, "1", 210);
  diagnostics->call(376, 82, 220);
  diagnostics->receive(1, 11, 376, 1, "1", 230);
  std::ostringstream summary;
  diagnostics->write_summary(summary);
  EXPECT_NE(summary.str().find("\"session\":2"), std::string::npos);
  EXPECT_NE(summary.str().find("\"last_replied_command_id\":82"), std::string::npos);
  EXPECT_NE(summary.str().find("\"previous_start_unreplied\":false"), std::string::npos);
}

TEST(ServoUdpDiagnostics, UnknownUdpOperationInvalidatesCounterPrediction)
{
  auto tracker = std::make_unique<ServoUdpTracker>();
  start(*tracker);
  tracker->call(ServoUdpCall{200, 376, 21});
  tracker->reply(reply(210, 1, 999));
  tracker->reply(reply(220, 2));
  EXPECT_TRUE(tracker->snapshot().degraded);
  EXPECT_EQ(tracker->snapshot().matched_replies, 0U);
}

TEST(ServoUdpDiagnostics, ReceiveCallbackStoresBoundedRawContentAndJsonEscapesIt)
{
  auto diagnostics = std::make_unique<ServoUdpDiagnostics>();
  ASSERT_TRUE(diagnostics->enable_once());
  diagnostics->call(689, -1, 100);
  const std::string content = "\"\n" + std::string(100, 'x');
  diagnostics->receive(1, 7, 689, static_cast<int>(content.size()), content, 110);
  std::ostringstream history;
  diagnostics->write_history(history);
  EXPECT_NE(history.str().find("\"content_truncated\":true"), std::string::npos);
  EXPECT_NE(history.str().find("\\\"\\u000a"), std::string::npos);
  EXPECT_EQ(history.str().find(std::string(64, 'x')), std::string::npos);
}

TEST(ServoUdpDiagnostics, SingleProducerQueuePreservesSequenceAcrossConcurrentDrains)
{
  fairino_hardware::ServoDiagnosticQueue<int, 128> queue;
  std::thread producer([&queue]() {
      for (int index = 1; index <= 10000; ++index) {
        while (!queue.push(index)) {std::this_thread::yield();}
      }
    });
  for (int index = 1; index <= 10000; ++index) {
    int value = 0;
    while (!queue.pop(value)) {std::this_thread::yield();}
    EXPECT_EQ(value, index);
  }
  producer.join();
}

}  // namespace
