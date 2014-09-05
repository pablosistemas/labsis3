/*
#include "../libraries/petri/arduino.h"
#include "../libraries/petri/gmatrix.h"
#include "../libraries/petri/petri.h"
 */

#include <arduino.h>
#include <gmatrix.h>
#include <petri.h>
#include <string.h>
#include <PID_v1.h> // Biblioteca do PID
//#include <TimerOne.h>

// rede de petri
PETRI net;

// Dimensões (M,N) da matriz A
#define M (13)
#define N (12)

// Matrizes da rede de petri


PETRI_T A[M * N] = {
                     0  , 0  , 0  , 0  , 1  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0,
                     0  , -1 , 0  , 1  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0,
                     0  , 1  , 0  , -1 , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0, 
                     -1 , 0  , 1  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0,
                     1  , 0  , -1 , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , 1  , 0  , -1 , 0  , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , -1 , 0  , 1  , 0  , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , 0  , 1  , 0  , -1 , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , 0  , -1 , 0  , 1  , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , -1 , -1 , 1  , 1,
                     0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1  , 0  , -1 , 0,
                     0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1  , 0  , -1
                   };
                     
PETRI_T Am[M * N] = {
		     0  , 1  , 0  , 1  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0,
                     0  , 1  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 1  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0,
                     1  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0,
                     0  , 0  , 1  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , 0  , 0  , 1  , 0  , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , 1  , 0  , 0  , 0  , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1  , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , 0  , 1  , 0  , 0  , 0  , 0  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1  , 1  , 0  , 0,
                     0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1  , 0,
                     0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 0  , 1
                   };
                    
PETRI_T X[1 * N] = {1  , 127  , 0  , 1  , 0  , 1  , 0  , 127 , 0  , 1  , 0  , 0 };

PETRI_T u[1 * M];

PETRI_T Xnew[1 * N];

PETRI_CMD C[M];

// fim das matrizes da rede de petri

// definições dos indexadores dos comandos e transições
#define ACENDER_QUARTO 0
#define ACENDER_SALA 1
#define APAGAR_QUARTO 2
#define APAGAR_SALA 3
#define ATIVA_ILUMINACAO_SALA 4
#define FECHAR_PORTAO 5
#define DESLIGAR_VENTO 6
#define ABRIR_PORTAO 7
#define LIGAR_VENTO 8
#define ELEVADOR_SOBE 9
#define ELEVADOR_DESCE 10
#define ELEVADOR_SUBIU_TUDO 11
#define ELEVADOR_DESCEU_TUDO 12

// definições dos indexadores dos lugares
#define ATIVA_LUZ_SALA 0
#define LUZES_SALA_DESLIGADAS 1
#define LUZES_SALA_LIGADAS 2
#define LUZES_QUARTO_DESLIGADAS 3
#define LUZES_QUARTO_LIGADAS 4
#define PORTAO_FECHADO 5
#define PORTAO_ABERTO 6
#define VENTILADOR_DESLIGADO 7
#define VENTILADOR_LIGADO 8
#define ELEVADOR_PARADO 9
#define ELEVADOR_SUBINDO 10
#define ELEVADOR_DESCENDO 11

// fim das definições

// variáveis do usuário
char acq = false; // liga/desliga luz do quarto
char sala = 2; // 0 - desligada, 1 - ligada, 2 - automático (LDR)

// entradas do arduino
// LDR
#define inLDR A0
#define LDR_IDEAL 700
#define LDR_CINEMA 250
#define LDR_OFFSET 10
#define ORDEM_FILTRO 5
int LDR_meta = LDR_IDEAL;
int LDR;
int filtro[ORDEM_FILTRO] = { 0 };

// Temperatura
#define inTemp A5
#define TEMP_IDEAL 59
int temp_meta = TEMP_IDEAL;
int temp = 0;

// saídas do arduino
// iluminação doquarto
#define outLedQuarto 9
unsigned char ledQuarto = (unsigned char) 0;

// iluminação da sala
#define outR 6
#define outG 5
#define outB 3

const PETRI_CMD* pR = &(X[LUZES_SALA_LIGADAS]);
#define R (2*(*pR))
#define G (2*(*pR)*0.7294)
#define B (2*(*pR)*0.1333)

// ventoinha
#define outVento 13
// unsigned char vento = (unsigned char) 0;

// portão
#define outPortao_PWM 11
#define outPortao_direcao1 12
#define outPortao_direcao2 4
#define inPortao A3
#define setPointFechado 450
#define setPointAberto 780
#define PTIMER 16384 // intervalo (us) para executar função de interrupção
int portao_direction = 0; //direção do portão (0 - fechar, 1 - abrir)
// unsigned char portao = (unsigned char) 0;

// Definição das entradas e saídas relativas ao controlador do portão:
double Portao_Setpoint, Portao_Input, Portao_Output;

// Definição do controlador do portão
PID PID_Portao(&Portao_Input, &Portao_Output, &Portao_Setpoint, 0.01, 0, 0, REVERSE);

// Definição das entradas e saídas relativas ao elevador
#define outElevador_PWM 10
#define outElevador_direcao1 8
#define outElevador_direcao2 7
#define inElevador_fimUp A2
#define inElevador_fimDown A3

// fim das entradas e saídas do arduino

void setup() {
	Serial.begin(9600);
	petriInit(&net, M, N, A, Am, X, C);

	// Setup da iluminação da sala
	pinMode(outR, OUTPUT);
	pinMode(outG, OUTPUT);
	pinMode(outB, OUTPUT);
	analogWrite(outR, *pR);
	analogWrite(outG, G);
	analogWrite(outB, B);

	// Setup da iluminação do quarto
	pinMode(outLedQuarto, OUTPUT);
	digitalWrite(outLedQuarto, X[LUZES_QUARTO_LIGADAS]);

	// Setup da ventoinha
	pinMode(outVento, OUTPUT);
	digitalWrite(outVento, X[VENTILADOR_LIGADO]);

	// Setup do portão
	pinMode(outPortao_PWM, OUTPUT);
        pinMode(outPortao_direcao1, OUTPUT);
        pinMode(outPortao_direcao2, OUTPUT);

        // Setup do controlador do portão
        Portao_Input = analogRead(A3);
        Portao_Setpoint = setPointFechado;
        // Liga o controlador do portão no automático
        PID_Portao.SetMode(AUTOMATIC);
        
        // Setup da função de interrupção do controle do portão
//        Timer1.initialize(PTIMER);
//        Timer1.attachInterrupt(pcontroler);
        
        // Setup do elevador (a saída inicial do PWM tem que ser 0)
        pinMode(outElevador_PWM, OUTPUT);
        digitalWrite(outElevador_PWM, X[ELEVADOR_SUBINDO]);
        
        // (a saída inicial da direção tanto faz...)
        pinMode(outElevador_direcao1, OUTPUT);
        pinMode(outElevador_direcao2, OUTPUT);
        digitalWrite(outElevador_PWM, X[ELEVADOR_SUBINDO]);
}

/*void pcontroler()
{
      Portao_Input = analogRead(A3); // lê valor do potenciômetro
      PID_Portao.Compute(); // roda controlador
      analogWrite(outPortao_PWM, Portao_Output); // escreve valor na saída
      digitalWrite(outPortao_direcao, portao_direction); // direção da ponte-H
}*/

void loop()
{
	// Reseta todos os comandos para desligados
	memset(C, false, sizeof(C));

	// Ler comandos do usuário
	if (Serial.available() > 0)
        {
		// read the incoming byte:
		char incomingByte = Serial.read();

		switch (incomingByte)
                {
		    case 'q':
	                acq = acq ? false : true;
			break;
		    case 'c':
			if (LDR_meta == LDR_CINEMA) LDR_meta = LDR_IDEAL;
			else
                        {
				LDR_meta = LDR_CINEMA;
				sala = 2; // coloca iluminação da sala no automático
			}
			break;
		    case 's':
			sala = sala == 0 ? 1 : sala == 1 ? 2 : 0;
			LDR_meta = LDR_IDEAL;
			break;
		    case 'p':
			if (Portao_Setpoint == setPointFechado)
                        {   // ajusta estados do controlador para abrir o portão
                            Portao_Setpoint = setPointAberto;
                            PID_Portao.SetTunings(3, 0.1, 0);
                            PID_Portao.SetControllerDirection(DIRECT);
                            portao_direction = 1;
                            C[ABRIR_PORTAO] = true;
                        }
                        else
                        {   // ajusta estados do controlador para fechar o portão
                            Portao_Setpoint = setPointFechado; 
                            PID_Portao.SetTunings(0.01, 0, 0);
                            PID_Portao.SetControllerDirection(REVERSE);
                            portao_direction = 0;
                            C[FECHAR_PORTAO] = true;
                        }
			break;
		    case 'v':
			temp_meta = temp_meta == TEMP_IDEAL ? 1024 : TEMP_IDEAL;
                        break;
                    case 'u':
                        C[ELEVADOR_SOBE] = true;
                        break;
                    case 'd':
                        C[ELEVADOR_DESCE] = true;
                        break;
                    case 'r':
                        // envia estados atuais ao aplicativo
                        // estado da sala
                        Serial.print(sala, DEC);
                        Serial.print('.'); // flag de fim de estado
                            
                        // modo cinema ativado
                        if (LDR_meta == LDR_CINEMA)
                        {
                            Serial.print('1'); // modo cinema ativado
                        }
                        else
                        {
                            Serial.print('0'); // modo cinema desativado
                        }
                        Serial.print('.'); // flag de fim de estado
                          
                        // luz da sala ligada
                        Serial.print(acq, DEC);
                        Serial.print('.'); // flag de fim de estado
                        
                        // ventilador ligado
                        if (temp_meta == TEMP_IDEAL)
                        {
                            Serial.print('1'); // ventilador ligado
                        }
                        else
                        {
                            Serial.print('0'); // ventilador desligado
                        }
                        Serial.print('.'); // flag de fim de estado
                        
                        // portão aberto
                        if (Portao_Setpoint == setPointAberto)
                        {
                            Serial.print('1'); // portão aberto
                        }
                        else
                        {
                            Serial.print('0'); // portão fechado
                        }
                        Serial.print('.'); // flag de fim de estado
                        break;
		}
	}

	// Lê sensor LDR
	memmove(filtro, &(filtro[1]), (ORDEM_FILTRO - 1)*sizeof(int));
	filtro[ORDEM_FILTRO - 1] = analogRead(A0);
	int sum = 0;
	for(int i = 0; i < ORDEM_FILTRO; ++i) {
		sum += filtro[i];
	}
	LDR = sum/ORDEM_FILTRO;

//	Serial.println("Controle da Janela");
//	Serial.println(C[ATIVA_CONT_JANELA], DEC);
	// Define ação da luz da sala
	// if (!C[ATIVA_CONT_JANELA]) { // Somente meche na iluminação se não for mecher na janela
            switch (sala)
            {
		case 0: // caso luz desligada
			if (X[LUZES_SALA_LIGADAS] != 0) {
				//C[ATIVA_ILUMINACAO_SALA] = true;
				C[APAGAR_SALA] = true;
			}
			break;
		case 1: // caso luz ligada
			if (X[LUZES_SALA_LIGADAS] != 127) {
				//C[ATIVA_ILUMINACAO_SALA] = true;
				C[ACENDER_SALA] = true;
			}
			break;
		case 2: // caso luz automática
			if (LDR != LDR_meta) {
				//C[ATIVA_ILUMINACAO_SALA] = true;
				if (LDR < (LDR_meta-LDR_OFFSET))
					C[ACENDER_SALA] = true;
				else if (LDR > (LDR_meta+LDR_OFFSET))
					C[APAGAR_SALA] = true;
			}
			break;
		}
//	}

	// Sensor de Temperatura do Quarto
	temp = analogRead(A5);
	if (temp <= temp_meta) { // caso esteja muito frio
		C[DESLIGAR_VENTO] = true;
	} else if (temp > temp_meta) { // caso esteja muito quente
		C[LIGAR_VENTO] = true;
	}

        // Lê sensores de fim de curso do elevador
        if (analogRead(A1) > 800) {
          C[ELEVADOR_SUBIU_TUDO] = true;
        }
        if (analogRead(A2) > 800) {
          C[ELEVADOR_DESCEU_TUDO] = true;
        }
        
	// Definir comandos com base em variáveis do usuário
	if (acq) C[ACENDER_QUARTO] = true;
	else C[APAGAR_QUARTO] = true;

        // atualiza a petri
	petriInit(&net, M, N, A, Am, X, C);
	petriAutoFire(&net, u, Xnew);
	//	petriPrintState(&net);

	// Atualizar saídas
	// Iluminação da sala
	analogWrite(outR, R);
	analogWrite(outG, G);
	analogWrite(outB, B);

	// Iluminação do quarto
	digitalWrite(outLedQuarto, X[LUZES_QUARTO_LIGADAS]);

	// Ventoinha
	digitalWrite(outVento, X[VENTILADOR_LIGADO]);

        // Portão
        Portao_Input = analogRead(A3); // lê valor do potenciômetro
        PID_Portao.Compute(); // roda controlador
        analogWrite(outPortao_PWM, Portao_Output); // escreve valor na saída
        digitalWrite(outPortao_direcao1, portao_direction); // direção da ponte-H
        digitalWrite(outPortao_direcao2, !portao_direction); // direção da ponte-H
        
        // Elevador
        digitalWrite(outElevador_PWM, !X[ELEVADOR_PARADO]);
        digitalWrite(outElevador_direcao1, !X[ELEVADOR_SUBINDO]);
        digitalWrite(outElevador_direcao2, X[ELEVADOR_SUBINDO]);
        
	// fim das atualizações das saídas
/*
        // impressões de teste (não funciona com bluetooth ativado, apenas no monitor do arduino)
        // sendor de temperatura
	Serial.print("Temperatura: ");
	Serial.println(temp);
        // sensor de iluminação
        Serial.print("LDR: ");
	Serial.println(LDR, DEC);
        // iluminação da sala
	Serial.print("R: ");
	Serial.println(R, DEC);
	Serial.print("G: ");
	Serial.println(G, DEC);
	Serial.print("B: ");
	Serial.println(B, DEC);
        for(int z=0;z<N;z++)
        {
            Serial.print(X[z], DEC);
            Serial.print(" ");
        }
        Serial.println(" ");
        delay(100); // atraso para exergar no display
        */
        
        
}

void arduinoPrintDec(const int a)
{
	Serial.print(a, DEC);
}

void arduinoPrintString(const char* s)
{
	Serial.print(s);
}


