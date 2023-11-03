char** morse_codes[26] = {
	"01",
	"1000",
	"1010",
	"100",
	"0",
	"0010",
	"110",
	"0000",
	"00",
	"1000",
	"101",
	"0100",
	"11",
	"10",
	"111",
	"0110",
	"1101",
	"010",
	"000",
	"1",
	"001",
	"0001",
	"011",
	"1001",
	"1011",
	"1100"
};


char* char_to_morze(char c){
	return morse_codes[(int)(c-'a')];
}

int str_to_morze(char* buf_in, int ptr_in, int* buf_out, int ptr_out){
	for(int i = 0; i < ptr_in; i++){
        char* c = str_to_morze(buf_in[i]);
        int len = strlen(c);
        for (int j = 0; j < len; j++){
			if(j > ptr_out){ // проверяем, что буфер не преполнился
				buf_out[j] = c - '0';
			}else return -1;
        }
    }
	return 1;
}

char morze_to_str(int b[], int p){
	if(b[0] == 0){
		if(p == 1) return 'e';
		if(b[1] == 0){
			if(p == 2) return 'i';
			if(b[2] == 0){
				if(p == 3) return 's';
				if(b[3] == 0){
					if(p == 4) return 'h';
				}else{
					if(p == 4) return 'v';
				}
				return '0';
			}else{
				if(p == 3) return 'u';
				if(b[3] == 0){
					if(p == 4) return 'f';
				}
				return '0';

			}
		}else{
			if(p == 2) return 'a';
			if(b[2] == 0){
				if(p == 3) return 'r';
				if(b[3] == 0){
					if(p == 4) return 'l';
				}
				return '0';
			}else{
				if(p == 3) return 'w';
				if(b[3] == 0){
					if(p == 4) return 'p';
				}else{
					if(p == 4) return 'j';
				}
				return '0';

			}
		}
	}else{
		if(p == 1) return 't'; //1
		if(b[1] == 0){
			if(p == 2) return 'n';//10
			if(b[2] == 0){
				if(p == 3) return 'd'; //100
				if(b[3] == 0){
					if(p == 4) return 'b'; //1000
				}else{
					if(p == 4) return 'x'; //1001
				}
				return '0';
			}else{
				if(p == 3) return 'k'; //101
				if(b[3] == 0){
					if(p == 4) return 'c'; //1010
				}else{
					if(p == 4) return 'y'; //1011
				}
				return '0';

			}
		}else{
			if(p == 2) return 'm'; //11
			if(b[2] == 0){
				if(p == 3) return 'g'; //110
				if(b[3] == 0){
					if(p == 4) return 'z'; //1100
				}
				return '0';
			}else{
				if(p == 3) return 'o'; //111
				return '0';

			}
		}

	}
}
