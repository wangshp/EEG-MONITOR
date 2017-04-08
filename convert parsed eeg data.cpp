#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <stdlib.h>
#include <cmath>
#include <bitset>

using namespace std;
int main() {
	//char* argve[];
	char* argv[] = { "", "config.txt", "signal.txt" };


	ifstream traces;
	ofstream tracesout;
	ofstream tracesout2;
	string outname, outname2;
	outname = string(argv[2]) + ".txt";
	outname2 = string(outname) + ".txt";

	traces.open(argv[2]);
	tracesout.open(outname.c_str());
	tracesout2.open(outname2.c_str());
	string line;
	string accesstype;  // the Read/Write access type from the memory trace;
	string xaddr;       // the address from the memory trace store in hex;
	string charactor;
	string charactor_value;
	string notify;
	string data_value1;
	string data_value2;
	string data_value3;
	string data_value4;
	string data_value5;
	string accesstype2;
	unsigned int addr;  // the address from the memory trace store in unsigned int;        
	bitset<32> accessaddr; // the address from the memory trace store in the bitset;

	unsigned long value_interger;
	

	if (traces.is_open() && tracesout.is_open() && tracesout2.is_open()) {
		while (getline(traces, line)) {   // read mem access file and access Cache

			istringstream iss(line);
			if (!(iss >> accesstype >> xaddr >> charactor >> charactor_value >> notify>> data_value1 >> data_value2 >> data_value3>> data_value4 >> data_value5)) { break; }
			stringstream saddr(data_value2);
			saddr >> std::hex >> addr;
			accessaddr = bitset<32>(addr);
			value_interger = accessaddr.to_ulong();
			//cout << data_value1 << " " << data_value2;
			//while (1)
			//	;
			char *time = &accesstype[0u];
			char *value1 = &data_value1[0u];
			char *value2 = &data_value2[0u];
			char *value3 = &data_value3[0u];
			char *value4 = &data_value4[0u];
			char *value5 = &data_value5[0u];

			tracesout << value1[1] << value1[2] << value1[3] << value1[4] << value1[5] << value1[6] << endl;
			tracesout << value1[7] << value1[8] << value2[0] << value2[1] << value2[2] << value2[3] << endl;
			tracesout << value2[4] << value2[5] << value2[6] << value2[7] << value3[0] << value3[1] << endl;
			tracesout << value3[2] << value3[3] << value3[4] << value3[5] << value3[6] << value3[7] << endl;
			tracesout << value4[0] << value4[1] << value4[2] << value4[3] << value4[4] << value4[5] << endl;
			tracesout << value4[6] << value4[7] << value5[0] << value5[1] << value5[2] << value5[3] << endl;

			//	tracesout << value_interger << endl;  // Output hit/miss results for L1 and L2 to the output file;
				tracesout2 << time[6] << time[7] << time[8] << time[9] << time[10] << time[11] << endl;
				
			
		}
		traces.close();
		tracesout.close();
		tracesout2.close();
	}
	else cout << "Unable to open trace or traceout file ";

	ifstream traces_2;
	ofstream tracesout_2;
	string outname3;
	outname3 = string(outname2) + ".txt";

	traces_2.open(outname);
	tracesout_2.open(outname3.c_str());

	string line2;
	string hexdata;
	unsigned int addr2;
	bitset<32> accessaddr2;
	unsigned long value_interger2;

	if (traces_2.is_open() && tracesout_2.is_open())
	{
		while (getline(traces_2, line2))
		{   // read mem access file and access Cache

			istringstream iss2(line2);
			if (!(iss2 >> hexdata)) { break; }
			stringstream saddr2(hexdata);
			saddr2 >> std::hex >> addr2;
			accessaddr2 = bitset<32>(addr2);
			value_interger2 = accessaddr2.to_ulong();

			tracesout_2 << value_interger2 << endl;
		}
		traces_2.close();
		tracesout_2.close();
	}
	else cout << "Unable to open trace or traceout file ";
	return 0;
}