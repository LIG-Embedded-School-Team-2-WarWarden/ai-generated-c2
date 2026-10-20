#include "c2/asset_registry.hpp"
#include "c2/connection_monitor.hpp"
#include "c2/telemetry_store.hpp"
#include "c2/attack_command_service.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
void check(bool ok,const char* detail){if(!ok)throw std::runtime_error(detail);}
int main(){
    using R=c2::AssetRegistryResult;
    c2::AssetRegistryConfig config;config.heartbeat_timeout_us=100;config.status_timeout_us=100;
    c2::AssetRegistry registry(config);
    c2::Endpoint source{"127.0.0.1",6001};
    c2::MessageHeader h{3,1,1000,c2::ComponentId::effector_asset,c2::ComponentId::command_and_control,201,123};
    c2::AssetRegistration registration{h,c2::AssetRole::effector,6001,c2::capability::effector_point,"sequence-test",{}, {-180,180,-90,90},false,5000};
    check(registry.register_asset(registration,source,1000)==R::registered,"register");
    c2::Heartbeat heartbeat{h,c2::AssetOperatingState::standby,1,1000};
    c2::EffectorStatus status;status.header=h;status.state=c2::EffectorState::standby;status.timestamp_us=1000;
    heartbeat.header.sequence=10;status.header.sequence=10;
    check(registry.observe_heartbeat(heartbeat,source,1000)==R::stored,"initial heartbeat");
    check(registry.update_effector_status(status,source,1000)==R::stored,"separate status sequence");
    // A changed payload/timestamp with the same sequence cannot refresh data.
    heartbeat.timestamp_us=2000;status.timestamp_us=2000;status.current_pan_deg=30;
    check(registry.observe_heartbeat(heartbeat,source,1080)==R::duplicate,"heartbeat duplicate");
    check(registry.update_effector_status(status,source,1080)==R::duplicate,"status duplicate");
    auto snapshot=registry.asset(201,1101);
    check(snapshot->connection_state==c2::AssetConnectionState::disconnected&&!snapshot->status_current&&
          snapshot->effector_status->current_pan_deg==0,"duplicates refreshed freshness/data");
    heartbeat.header.sequence=9;status.header.sequence=9;
    check(registry.observe_heartbeat(heartbeat,source,1102)==R::stale,"old heartbeat with newer timestamp");
    check(registry.update_effector_status(status,source,1102)==R::stale,"old status with newer timestamp");
    // New sequence is accepted despite a backwards wall clock.
    heartbeat.header.sequence=11;status.header.sequence=11;heartbeat.timestamp_us=500;status.timestamp_us=500;
    heartbeat.header.timestamp_us=status.header.timestamp_us=500;
    check(registry.observe_heartbeat(heartbeat,source,1103)==R::stored,"clock rollback heartbeat");
    check(registry.update_effector_status(status,source,1103)==R::stored,"clock rollback status");
    check(registry.asset(201,1103)->status_current,"new status not current");
    // A registration refresh in the same session must not reset watermarks.
    check(registry.register_asset(registration,source,1104)==R::refreshed,"refresh registration");
    heartbeat.header.sequence=status.header.sequence=1;
    check(registry.observe_heartbeat(heartbeat,source,1105)==R::stale,"refresh reset heartbeat watermark");
    check(registry.update_effector_status(status,source,1105)==R::stale,"refresh reset status watermark");
    // New session resets each stream; old-session packets remain rejected.
    registration.header.session_id=124;
    check(registry.register_asset(registration,source,1106)==R::session_replaced,"new session");
    check(registry.observe_heartbeat(heartbeat,source,1107)==R::session_mismatch,"old heartbeat session");
    check(registry.update_effector_status(status,source,1107)==R::session_mismatch,"old status session");
    heartbeat.header.session_id=status.header.session_id=124;
    heartbeat.header.sequence=status.header.sequence=std::numeric_limits<std::uint32_t>::max();
    check(registry.observe_heartbeat(heartbeat,source,1108)==R::stored,"new session heartbeat");
    check(registry.update_effector_status(status,source,1108)==R::stored,"new session status");
    heartbeat.header.sequence=status.header.sequence=1;
    check(registry.observe_heartbeat(heartbeat,source,1109)==R::stored,"heartbeat wrap");
    check(registry.update_effector_status(status,source,1109)==R::stored,"status wrap");
    heartbeat.header.sequence=status.header.sequence=std::numeric_limits<std::uint32_t>::max();
    check(registry.observe_heartbeat(heartbeat,source,1110)==R::stale,"prewrap heartbeat replay");
    check(registry.update_effector_status(status,source,1110)==R::stale,"prewrap status replay");
    heartbeat.header.sequence=status.header.sequence=0x80000001U;
    check(registry.observe_heartbeat(heartbeat,source,1111)==R::stale,"ambiguous heartbeat half-range");
    check(registry.update_effector_status(status,source,1111)==R::stale,"ambiguous status half-range");
    c2::ConnectionMonitor monitor({100});
    c2::TelemetryStore telemetry(8);
    c2::AttackCommandConfig attack_config;attack_config.command_validity_us=1000;
    c2::AttackCommandService safety(attack_config);
    heartbeat.header.sequence=status.header.sequence=10;
    heartbeat.timestamp_us=status.timestamp_us=1000;
    check(monitor.observe(heartbeat,1000)==c2::HeartbeatUpdateResult::stored,"legacy heartbeat initial");
    check(telemetry.update(status)==c2::TelemetryUpdateResult::stored,"legacy status initial");
    check(safety.update_status(status)==c2::AttackStatusUpdateResult::stored,"legacy safety initial");
    heartbeat.timestamp_us=status.timestamp_us=2000;
    check(monitor.observe(heartbeat,1080)==c2::HeartbeatUpdateResult::duplicate,"legacy heartbeat duplicate");
    check(telemetry.update(status)==c2::TelemetryUpdateResult::duplicate,"legacy status duplicate");
    check(safety.update_status(status)==c2::AttackStatusUpdateResult::duplicate,"legacy safety duplicate");
    check(monitor.state(c2::ComponentId::effector_asset,1101)==c2::ConnectionState::disconnected,"legacy duplicate extended connection");
    heartbeat.header.sequence=status.header.sequence=9;
    check(monitor.observe(heartbeat,1102)==c2::HeartbeatUpdateResult::out_of_order,"legacy disconnected watermark reset");
    check(telemetry.update(status)==c2::TelemetryUpdateResult::out_of_order,"legacy status order");
    check(safety.update_status(status)==c2::AttackStatusUpdateResult::out_of_order,"legacy safety order");
    heartbeat.header.sequence=status.header.sequence=11;
    heartbeat.timestamp_us=status.timestamp_us=400;
    check(monitor.observe(heartbeat,1103)==c2::HeartbeatUpdateResult::stored,"legacy clock rollback heartbeat");
    check(telemetry.update(status)==c2::TelemetryUpdateResult::stored,"legacy clock rollback status");
    check(safety.update_status(status)==c2::AttackStatusUpdateResult::stored,"legacy clock rollback safety");
    std::cout<<"Sequence passed: independent streams, duplicates/reordering, clock rollback, freshness, sessions, wrap/half-range\n";
}
