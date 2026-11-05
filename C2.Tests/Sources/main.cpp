#include <iostream>
int rdr_protocol_entry();int rdr_icd_contract_entry();int fcs_client_entry();
int gui_v3_entry();
int rdr_live_diagnostics_entry();
int main(){try{if(rdr_live_diagnostics_entry()||gui_v3_entry()||rdr_protocol_entry()||rdr_icd_contract_entry()||fcs_client_entry())return 1;
std::cout<<"PASS: binary RDR/FCS ICD tests\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
