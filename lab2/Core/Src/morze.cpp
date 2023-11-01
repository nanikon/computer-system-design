
#include <map>
using namespace std;

typedef struct{
	int* arr;
	int len;
}Morze;

map<char, Morze> morze =
		{
		 {'a', {new int[2]{0, 1}, 2}},
		 {'b', {new int[4]{1, 0, 0, 0}, 4}},
		 {'c', {new int[4]{1, 0, 1, 0}, 4}},
		 {'d', {new int[3]{1, 0, 0}, 3}},
		 {'e', {new int[1]{0}, 1}},
		 {'f', {new int[4]{0, 0, 1, 0}, 4}},
		 {'g', {new int[3]{1, 1, 0}, 3}},
		 {'h', {new int[4]{0, 0, 0, 0}, 4}},
		 {'i', {new int[2]{0, 0}, 2}},
		 {'j', {new int[4]{1, 0, 0, 0}, 4}},
		 {'k', {new int[3]{1, 0, 1}, 3}},
		 {'l', {new int[4]{0, 1, 0, 0}, 4}},
		 {'m', {new int[2]{1, 1}, 2}},
		 {'n', {new int[2]{1, 0}, 2}},
		 {'o', {new int[3]{1, 1, 1}, 3}},
		 {'p', {new int[4]{0, 1, 1, 0}, 4}},
		 {'q', {new int[4]{1, 1, 0, 1}, 4}},
		 {'r', {new int[3]{0, 1, 0}, 3}},
		 {'s', {new int[3]{0, 0, 0}, 3}},
		 {'t', {new int[1]{1}, 1}},
		 {'u', {new int[3]{0, 0, 1}, 3}},
		 {'v', {new int[4]{0, 0, 0, 1}, 4}},
		 {'w', {new int[3]{0, 1, 1}, 3}},
		 {'x', {new int[4]{1, 0, 0, 1}, 4}},
		 {'y', {new int[4]{1, 0, 1, 1}, 4}},
		 {'z', {new int[4]{1, 1, 0, 0}, 4}}
		};
Morze str_to_morze(char c){
	return morze[c];
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
