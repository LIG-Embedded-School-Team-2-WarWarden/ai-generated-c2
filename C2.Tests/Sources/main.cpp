#include <iostream>
#include <string_view>
int rdr_protocol_entry();int rdr_icd_contract_entry();int fcs_client_entry();
int gui_v3_entry();
int lidar_interlock_entry();
int assignment_position_guard_entry();
int rdr_live_diagnostics_entry();
int gui_position_filter_entry();
int target_gap_predictor_entry();
int gui_filter_replay_entry(const char*,const char*);
int main(int argc,char**argv){try{
if(argc==4 && std::string_view(argv[1])=="--replay-gui-filter")return gui_filter_replay_entry(argv[2],argv[3]);
if(argc!=1)throw std::runtime_error("usage: C2.Tests [--replay-gui-filter INPUT.json OUTPUT.json]");
if(lidar_interlock_entry()||target_gap_predictor_entry()||assignment_position_guard_entry()||gui_position_filter_entry()||rdr_live_diagnostics_entry()||gui_v3_entry()||rdr_protocol_entry()||rdr_icd_contract_entry()||fcs_client_entry())return 1;
std::cout<<"PASS: binary RDR/FCS ICD tests\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
