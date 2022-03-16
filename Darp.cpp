/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cFiles/file.c to edit this template
 */

#include <iostream>
#include <memory.h>
#include <math.h>

#include "darp.hpp"

#define MAX(X, Y) ((X > Y) ? X : Y)

// pesos para os termos da FO
const int peso_vei = 10;
const int peso_dst = 1;
const int peso_dur = 0;
const int peso_via = 0;
const int peso_esp = 0;

// pesos para as violações não tratadas na modelagem
const int peso_vio_cap_vei = 1000;
const int peso_vio_dur_rot = 100;
const int peso_vio_jan_tem = 100;
const int peso_vio_tem_via = 100;
const int peso_vio_tem_esp = 100;

using namespace std;

/*
   instancia
   local de saida -> "", nome do arquivo
*/
// 0 1 2
int main(int argc, char *argv[])
{
   solucao sol;
   string saida;
   string aux(argv[1]);

   string instancia = "instancias/";
   instancia += aux;

   // "" para mostrar na tela
   if(argc < 2){
      saida = "";
   }else{
      aux = string(argv[2]);
      saida = "resultados/";
      saida += aux;
   }
    
   lerDados(instancia);
   // testarDados("");
   ordenarLocais();
   // for (int i = 0; i < numLoc - 2; i++)
   //    printf("%d\n", vetLocOrd[i]);
   heuConGul(sol);
   calcFO(sol);

   simulatedAnnealing(0.005, 1000, 1000, 0.05, sol);
   calcFO(sol);
   escArquivo(sol, saida);
   escProblema(saida);
   escSolucao(sol, saida);
   return 0;
}

void calcFO(solucao &s)
{
   int orig, dest, carga;
   // FO
   s.numVeiUsa = 0;
   s.dstTotal = 0;
   s.durTotal = 0;
   s.tViTotal = 0;
   s.tEsTotal = 0;
   // programação das rotas
   memset(&s.vetCarVeiLoc, 0, sizeof(s.vetCarVeiLoc));
   memset(&s.vetVioCapVei, 0, sizeof(s.vetVioCapVei));
   for (int i = 0; i < numVei; i++)
   {
      carga = 0;
      s.vetDstPerVei[i] = 0;
      s.vetDurRotVei[i] = 0;
      if (s.vetQtdLocAte[i] > 0)
      {
         // saída da garagem
         orig = 0;
         carga += vetQtdAssLoc[orig];
         s.vetCarVeiLoc[orig] = carga;
         dest = s.matAteVei[i][0];
         s.vetDstPerVei[i] += matTemDesLoc[orig][dest];
         s.vetHorSaiVei[i] = MAX(vetIniJTLoc[orig], vetIniJTLoc[dest] - matTemDesLoc[orig][dest] - vetTemSerLoc[orig]);
         s.vetHorCheLoc[dest] = s.vetHorSaiVei[i] + vetTemSerLoc[orig] + matTemDesLoc[orig][dest];
         s.vetHorIniAte[dest] = MAX(s.vetHorCheLoc[dest], vetIniJTLoc[dest]);
         s.vetHorSaiLoc[dest] = s.vetHorIniAte[dest] + vetTemSerLoc[dest];
         s.vetTemEspLoc[dest] = s.vetHorIniAte[dest] - s.vetHorCheLoc[dest];
         carga += vetQtdAssLoc[dest];
         s.vetCarVeiLoc[dest] = carga;
         s.vetVioCapVei[i] += MAX(0, carga - vetCapVei[i]);
         orig = dest;
         // locais intermediários
         for (int j = 1; j < s.vetQtdLocAte[i]; j++)
         {
            dest = s.matAteVei[i][j];
            s.vetDstPerVei[i] += matTemDesLoc[orig][dest];
            s.vetHorCheLoc[dest] = s.vetHorSaiLoc[orig] + matTemDesLoc[orig][dest];
            s.vetHorIniAte[dest] = MAX(s.vetHorCheLoc[dest], vetIniJTLoc[dest]);
            s.vetHorSaiLoc[dest] = s.vetHorIniAte[dest] + vetTemSerLoc[dest];
            s.vetTemEspLoc[dest] = s.vetHorIniAte[dest] - s.vetHorCheLoc[dest];
            carga += vetQtdAssLoc[dest];
            s.vetCarVeiLoc[dest] = carga;
            s.vetVioCapVei[i] += MAX(0, carga - vetCapVei[i]);
            orig = dest;
         }
         // chegada na garagem
         dest = numLoc - 1;
         carga += vetQtdAssLoc[dest];
         s.vetCarVeiLoc[dest] = carga;
         s.vetVioCapVei[i] += MAX(0, carga - vetCapVei[i]);
         s.vetDstPerVei[i] += matTemDesLoc[orig][dest];
         s.vetHorCheVei[i] = MAX(vetIniJTLoc[dest], s.vetHorSaiLoc[orig] + matTemDesLoc[orig][dest] + vetTemSerLoc[dest]);
         // dados referentes aos veículos usados na FO
         s.numVeiUsa++;
         s.vetDurRotVei[i] = s.vetHorCheVei[i] - s.vetHorSaiVei[i];
         s.dstTotal += s.vetDstPerVei[i];
         s.durTotal += s.vetDurRotVei[i];
      }
   }
   // tempo de viagem
   s.tViTotal = 0;
   for (int i = 1; i <= numReq; i++)
   {
      s.vetTemViaReq[i] = s.vetHorIniAte[i + numReq] - s.vetHorSaiLoc[i];
      s.tViTotal += s.vetTemViaReq[i];
   }
   // tempo de espera
   memset(&s.vetTemEspReq, 0, sizeof(s.vetTemEspReq));
   for (int i = 0; i < numVei; i++)
   {
      for (int j = 0; j < s.vetQtdLocAte[i] - 1; j++)
      {
         if (s.matAteVei[i][j] <= numReq)
         {
            orig = s.matAteVei[i][j];
            for (int k = j + 1; k < s.vetQtdLocAte[i]; k++)
            {
               dest = s.matAteVei[i][k];
               s.vetTemEspReq[orig] += s.vetTemEspLoc[dest];
               if (dest == (orig + numReq))
                  break;
            }
         }
      }
   }
   for (int i = 1; i <= numReq; i++)
      s.tEsTotal += s.vetTemEspReq[i];
   // cálculo da FO
   s.funObj = peso_vei * s.numVeiUsa + peso_dst * s.dstTotal + peso_dur * s.durTotal +
              peso_via * s.tViTotal + peso_esp * s.tEsTotal;
   // cálculo violações não tratadas na modelagem
   s.vioCapVei = 0;
   s.vioDurRot = 0;
   s.vioJanTem = 0;
   s.vioTemVia = 0;
   s.vioTemEsp = 0;
   for (int i = 0; i < numVei; i++)
      if (s.vetQtdLocAte[i] > 0)
      {
         s.vioCapVei += s.vetVioCapVei[i];
         s.vetVioDurRot[i] = MAX(0, s.vetDurRotVei[i] - durMaxRot);
         s.vioDurRot += s.vetVioDurRot[i];
         s.vioJanTem += MAX(0, s.vetHorCheVei[i] - vetFinJTLoc[numLoc - 1]);
      }
   for (int i = 1; i <= numReq; i++)
   {
      s.vioTemVia += MAX(0, s.vetTemViaReq[i] - temMaxVia);
      s.vioTemEsp += MAX(0, s.vetTemEspReq[i] - temMaxEsp);
   }
   for (int i = 1; i < numLoc - 1; i++)
      s.vioJanTem += MAX(0, s.vetHorCheLoc[i] - vetFinJTLoc[i]);
   // atualização da FO
   s.funObj += peso_vio_cap_vei * s.vioCapVei + peso_vio_dur_rot * s.vioDurRot +
               peso_vio_jan_tem * s.vioJanTem + peso_vio_tem_via * s.vioTemVia +
               peso_vio_tem_esp * s.vioTemEsp;
}

void heuConGul(solucao &s)
{
   int vei, aux;
   memset(&s.vetQtdLocAte, 0, sizeof(s.vetQtdLocAte));
   vei = 0;
   for (int i = 0; i < numLoc - 2; i++)
   {
      if (vetLocOrd[i] > numReq)
      {
         aux = idVeiEmbLoc(s, vetLocOrd[i]);
         s.matAteVei[aux][s.vetQtdLocAte[aux]] = vetLocOrd[i];
         s.vetQtdLocAte[aux]++;
         if (aux == vei)
            vei++;
      }
      else
      {
         s.matAteVei[vei][s.vetQtdLocAte[vei]] = vetLocOrd[i];
         s.vetQtdLocAte[vei]++;
         vei++;
      }
      if (vei == numVei)
         vei = 0;
   }
}

void escSolucao(solucao &s, string arq)
{
   int loc;
   FILE *f;
   if (arq == "")
      f = stdout;
   else
      f = fopen(arq.c_str(), "a");
   fprintf(f, "\n\n< --------------------- SOLUCAO --------------------- >\n\n");
   fprintf(f, "Numero de veiculos utilizados..................: %d\n", s.numVeiUsa);
   fprintf(f, "Distancia total percorrida.....................: %d\n", s.dstTotal);
   fprintf(f, "Duracao total das rotas........................: %d minutos!\n", s.durTotal);
   fprintf(f, "Tempo total de viagem..........................: %d minutos!\n", s.tViTotal);
   fprintf(f, "Tempo total de espera..........................: %d minutos!\n", s.tEsTotal);
   fprintf(f, "Violacoes nas capacidades dos veiculos.........: %d (un)\n", s.vioCapVei);
   fprintf(f, "Violacoes na duracao das rotas.................: %d minutos!\n", s.vioDurRot);
   fprintf(f, "Violacoes nas janelas de tempo.................: %d minutos!\n", s.vioJanTem);
   fprintf(f, "Violacoes no tempo maximo de viagem............: %d minutos!\n", s.vioTemVia);
   fprintf(f, "Violacoes no tempo maximo de espera............: %d minutos!\n", s.vioTemEsp);
   fprintf(f, "Custo total da solucao (FO)....................: %d\n", s.funObj);
   //---
   fprintf(f, "\n\nVEICULOS:\n");
   for (int i = 0; i < numVei; i++)
      escVeiculo(s, i, f);
   //---
   fprintf(f, "\n\nREQUISICOES (PPNEs):\n");
   for (int i = 1; i <= numReq; i++)
      escRequisicao(s, i, f);
   //---
   fprintf(f, "\n\nPROGRAMACAO DETALHADA:\n");
   fprintf(f, "\nLegenda:\n   HC: Horario de chegada (min)\n   TE: Tempo de espera    (min)\n   IA: Inicio do atend.   (min)\n   HS: Horario de saida   (min)\n   TV: Tempo de viagem    (min)\n   CV: Carga do veiculo   (un)\n");
   for (int i = 0; i < numVei; i++)
   {
      if (s.vetQtdLocAte[i] > 0)
      {
         fprintf(f, "\nVeiculo %d\n", i + 1);
         fprintf(f, "Loc: %d\tHC: -\tTE: -\tIA: %d\tHS: %d\tTV: -\tCV: %d\n",
                 0, s.vetHorSaiVei[i], s.vetHorSaiVei[i] + vetTemSerLoc[0], vetQtdAssLoc[0]);
         for (int j = 0; j < s.vetQtdLocAte[i]; j++)
         {
            loc = s.matAteVei[i][j];
            if (loc > numReq)
            {
               fprintf(f, "Loc: %d\tHC: %d\tTE: %d\tIA: %d\tHS: %d\tTV: %d\tCV: %d\n",
                       loc, s.vetHorCheLoc[loc], s.vetTemEspLoc[loc], s.vetHorIniAte[loc], s.vetHorSaiLoc[loc], s.vetTemViaReq[loc - numReq], s.vetCarVeiLoc[loc]);
            }
            else
            {
               fprintf(f, "Loc: %d\tHC: %d\tTE: %d\tIA: %d\tHS: %d\tTV: -\tCV: %d\n",
                       loc, s.vetHorCheLoc[loc], s.vetTemEspLoc[loc], s.vetHorIniAte[loc], s.vetHorSaiLoc[loc], s.vetCarVeiLoc[loc]);
            }
         }
         fprintf(f, "Loc: %d\tHC: %d\tTE: -\tIA: -\tHS: -\tTV: -\tCV: %d\n",
                 numLoc - 1, s.vetHorCheVei[i], 0);
      }
   }
   if (arq != "")
      fclose(f);
}

void escRequisicao(solucao &s, const int req, FILE *f)
{
   fprintf(f, "\n>>> Requisicao %d:\n", req);
   fprintf(f, "Tempo de viagem...: %d\n", s.vetTemViaReq[req]);
   fprintf(f, "Tempo de espera...: %d\n", s.vetTemEspReq[req]);
}

void escVeiculo(solucao &s, const int vei, FILE *f)
{
   fprintf(f, "\n>>> Veiculo %d:\n", vei + 1);
   fprintf(f, "Numero de PPNEs atendidas............: %d\n", s.vetQtdLocAte[vei] / 2);
   fprintf(f, "Distancia percorrida.................: %d\n", s.vetDstPerVei[vei]);
   fprintf(f, "Duracao da rota......................: %d\n", s.vetDurRotVei[vei]);
   fprintf(f, "Violacoes na capacidade do veiculo...: %d (un)\n", s.vetVioCapVei[vei]);
   fprintf(f, "Violacoes na duracao da rota.........: %d minutos!\n", s.vetVioDurRot[vei]);
   fprintf(f, "Sequencia de locais...: 0 --> ");
   for (int j = 0; j < s.vetQtdLocAte[vei]; j++)
      fprintf(f, "%d --> ", s.matAteVei[vei][j]);
   fprintf(f, "%d\n", numLoc - 1);
}

void escProblema(string arq)
{
   FILE *f;
   if (arq == "")
      f = stdout;
   else
      f = fopen(arq.c_str(), "a");
   fprintf(f, "\n\n< ---------------------- DARP ----------------------- >\n\n");
   fprintf(f, "Numero de veiculos disponiveis.................: %d\n", numVei);
   fprintf(f, "Numero de requisicoes (PPNEs)..................: %d\n", numReq);
   fprintf(f, "Numero de localidades..........................: %d\n", numLoc);
   fprintf(f, "Duracao maxima de uma rota.....................: %d minutos!\n", durMaxRot);
   fprintf(f, "Tempo maximo de viagem.........................: %d minutos!\n", temMaxVia);
   fprintf(f, "Tempo maximo de espera.........................: %d minutos!\n", temMaxEsp);
   fprintf(f, "\n");
   fprintf(f, "Peso para o numero de veiculos utilizados......: %d\n", peso_vei);
   fprintf(f, "Peso para a distancia total percorrida.........: %d\n", peso_dst);
   fprintf(f, "Peso para a duracao da rota....................: %d\n", peso_dur);
   fprintf(f, "Peso para o tempo total de viagem..............: %d\n", peso_via);
   fprintf(f, "Peso para o tempo total de espera..............: %d\n", peso_esp);
   fprintf(f, "\n");
   fprintf(f, "Peso para violacoes na capacidade dos veiculos.: %d\n", peso_vio_cap_vei);
   fprintf(f, "Peso para violacoes na duracao das rotas.......: %d\n", peso_vio_dur_rot);
   fprintf(f, "Peso para violacoes nas janelas de tempo.......: %d\n", peso_vio_jan_tem);
   fprintf(f, "Peso para violacoes no tempo de viagem.........: %d\n", peso_vio_tem_via);
   fprintf(f, "Peso para violacoes no tempo de espera.........: %d\n", peso_vio_tem_esp);
   if (arq != "")
      fclose(f);
}

void escArquivo(solucao &s, string arq)
{
   FILE *f;
   if (arq == "")
      f = stdout;
   else
      f = fopen(arq.c_str(), "w");
   fprintf(f, "%d\t%d\n", s.funObj, s.numVeiUsa);
   for (int i = 0; i < numVei; i++)
      if (s.vetQtdLocAte[i] > 0)
      {
         fprintf(f, "%d\t%d\t", i, s.vetQtdLocAte[i] / 2);
         for (int j = 0; j < s.vetQtdLocAte[i]; j++)
            fprintf(f, "%d  ", s.matAteVei[i][j]);
         fprintf(f, "\n");
      }
   if (arq != "")
      fclose(f);
}

int idVeiEmbLoc(solucao &s, const int locDes)
{
   for (int i = 0; i < numVei; i++)
      for (int j = 0; j < s.vetQtdLocAte[i]; j++)
         if (s.matAteVei[i][j] == (locDes - numReq))
            return i;
   return -1;
}

void ordenarLocais()
{
   int flag, aux;
   for (int i = 0; i < numLoc - 2; i++)
      vetLocOrd[i] = i + 1;
   flag = 1;
   while (flag)
   {
      flag = 0;
      for (int j = 0; j < numLoc - 3; j++)
      {
         if (vetFinJTLoc[vetLocOrd[j]] > vetFinJTLoc[vetLocOrd[j + 1]])
         {
            flag = 1;
            aux = vetLocOrd[j];
            vetLocOrd[j] = vetLocOrd[j + 1];
            vetLocOrd[j + 1] = aux;
         }
      }
   }
}

void testarDados(string arq)
{
   FILE *f;
   if (arq == "")
      f = stdout;
   else
      f = fopen(arq.c_str(), "w");
   fprintf(f, "%d\t%d\t%d\t%d\t%d\n\n", numVei, numReq, durMaxRot, temMaxVia, temMaxEsp);
   numLoc = numReq * 2 + 2;
   for (int i = 0; i < numVei; i++)
      fprintf(f, "%d\t", vetCapVei[i]);
   fprintf(f, "\n\n");
   for (int i = 0; i < numLoc; i++)
      fprintf(f, "%d\t", vetQtdAssLoc[i]);
   fprintf(f, "\n\n");
   for (int i = 0; i < numLoc; i++)
      fprintf(f, "%d\t", vetTemSerLoc[i]);
   fprintf(f, "\n\n");
   for (int i = 0; i < numLoc; i++)
      fprintf(f, "%d\t", vetIniJTLoc[i]);
   fprintf(f, "\n\n");
   for (int i = 0; i < numLoc; i++)
      fprintf(f, "%d\t", vetFinJTLoc[i]);
   fprintf(f, "\n\n");
   for (int i = 0; i < numLoc; i++)
   {
      for (int j = 0; j < numLoc; j++)
         fprintf(f, "%d\t", matTemDesLoc[i][j]);
      fprintf(f, "\n");
   }
   if (arq != "")
      fclose(f);
}

void lerDados(string arq)
{
   FILE *f = fopen(arq.c_str(), "r");
   fscanf(f, "%d %d %d %d %d", &numVei, &numReq, &durMaxRot, &temMaxVia, &temMaxEsp);
   numLoc = numReq * 2 + 2;
   for (int i = 0; i < numVei; i++)
      fscanf(f, "%d", &vetCapVei[i]);
   for (int i = 0; i < numLoc; i++)
      fscanf(f, "%d", &vetQtdAssLoc[i]);
   for (int i = 0; i < numLoc; i++)
      fscanf(f, "%d", &vetTemSerLoc[i]);
   for (int i = 0; i < numLoc; i++)
      fscanf(f, "%d", &vetIniJTLoc[i]);
   for (int i = 0; i < numLoc; i++)
      fscanf(f, "%d", &vetFinJTLoc[i]);
   for (int i = 0; i < numLoc; i++)
      for (int j = 0; j < numLoc; j++)
         fscanf(f, "%d", &matTemDesLoc[i][j]);
   fclose(f);
}

void ordena(int *vectorToSort, int *timeVector, int size)
{
   int i, j, lowerValue;
   for (i = 0; i < size; i++)
   {
      lowerValue = i;
      for (j = i + 1; j < size; j++)
      {
         if (timeVector[vectorToSort[lowerValue]] > timeVector[vectorToSort[j]])
         {
            lowerValue = j;
         }
      }
      if (lowerValue != i)
      {
         j = vectorToSort[i];
         vectorToSort[i] = vectorToSort[lowerValue];
         vectorToSort[lowerValue] = j;
      }
   }
}

void mudar(solucao &s, int aux1, int aux2)
{
   int l1 = -1;
   int l2 = -1;
   int l3 = -1;
   int l4 = -1;
   int c1 = -1;
   int c2 = -1;
   int c3 = -1;
   int c4 = -1;

   int aux;

   if (aux1 > numReq)
   {
      aux1 -= numReq;
   }

   if (aux2 > numReq)
   {
      aux2 -= numReq;
   }

   for (int j = 0; j < s.numVeiUsa; j++)
   { // veiculos
      for (int k = 0; k < s.vetQtdLocAte[j]; k++)
      { // locais
         if (s.matAteVei[j][k] == aux1)
         {
            l1 = j;
            c1 = k;
         }
         if (s.matAteVei[j][k] == aux1 + numReq)
         {
            l2 = j;
            c2 = k;
         }
         if (s.matAteVei[j][k] == aux2)
         {
            l3 = j;
            c3 = k;
         }
         if (s.matAteVei[j][k] == aux2 + numReq)
         {
            l4 = j;
            c4 = k;
         }
      }
   }

   if (l1 == -1 || l2 == -1 || l3 == -1 || l4 == -1 || c1 == -1 || c2 == -1 || c3 == -1 || c4 == -1 || l1 != l2 || l3 != l4)
   {
      /*if (l1 == -1 || l2 == -1 || l3 == -1 || l4 == -1 || c1 == -1 || c2 == -1 || c3 == -1 || c4 == -1)
      {
         printf("Erro negativo");
         printf("\n%d  %d\n", aux1, aux2);
      }
      else
      {

         printf("Erro de posição\n");
      }
      escSolucao(s, "");
      printf("erro\n");
      */
     printf("Erro");
   }
   else
   {
      aux = s.matAteVei[l1][c1];
      s.matAteVei[l1][c1] = s.matAteVei[l3][c3];
      s.matAteVei[l3][c3] = aux;

      aux = s.matAteVei[l2][c2];
      s.matAteVei[l2][c2] = s.matAteVei[l4][c4];
      s.matAteVei[l4][c4] = aux;

      ordena(s.matAteVei[l1], vetFinJTLoc, s.vetQtdLocAte[l1]);
      ordena(s.matAteVei[l3], vetFinJTLoc, s.vetQtdLocAte[l3]);
   }
}

void mover(solucao &s, int requisicao, int carro)
{
   int i, j, k;
   if (requisicao > numReq)
   {
      requisicao -= numReq;
   }

   for (j = 0; j < s.numVeiUsa; j++)
   { // veiculos
      for (k = 0; k < s.vetQtdLocAte[j]; k++)
      {// locais
         if (s.matAteVei[j][k] == requisicao || s.matAteVei[j][k] == requisicao + numReq)
         {
            if (j == carro)
            {
               return;
            }
            else
            {
               for (i = k + 1; i <= s.vetQtdLocAte[j]; i++)
               {
                  s.matAteVei[j][i - 1] = s.matAteVei[j][i];
               }
               (s.vetQtdLocAte[j])--;
               //k = k != 0 ? k - 1 : k;
               k--;
            }
         }
      }
   }

   s.matAteVei[carro][s.vetQtdLocAte[carro]] = requisicao;
   s.vetQtdLocAte[carro] += 1;
   s.matAteVei[carro][s.vetQtdLocAte[carro]] = requisicao + numReq;
   s.vetQtdLocAte[carro] += 1;

   // ordena(s.matAteVei[carro], p->inicioJanelaTempo, s.vetQtdLocAte[carro]);
}

void geraVizinhoAleatorio(solucao &s, int qtd)
{
   int i, j, k;
   int aux1, aux2;
   for (i = 0; i < qtd; i++)
   {
      do
      {
         aux1 = rand() % numReq;
      } while (aux1 <= 0);

      do
      {
         aux2 = rand() % numReq;
      } while (aux2 == aux1 || aux2 <= 0);

      mudar(s, aux1, aux2);
   }
}

void gerarVizinhoMudandoDeCarro(solucao &s, int qtd)
{
   int requisicao;
   int carro;
   for (int i = 0; i < qtd; i++)
   {
      requisicao = (rand() % numReq) + 1; // 1 -- nReq
      carro = rand() % s.numVeiUsa;       // 0 -- (veiculos - 1)
      printf("%d     %d\n", requisicao, carro);
      mover(s, requisicao, carro);
   }
}

void simulatedAnnealing(double taxaResfriamento, int numeroDeInteracoes, double temperaturaInicial,
                        double temperaturaFinal, solucao &s1)
{

   solucao Ms;
   memcpy(&Ms, &s1, sizeof(solucao));
   solucao s2;
   int iterT = 0;
   double temperatura = temperaturaInicial;
   int variacao;
   double x;

   double E = 2.71828;

   while (temperatura > temperaturaFinal)
   {

      while (iterT < numeroDeInteracoes)
      {
         iterT++;
         memcpy(&s2, &s1, sizeof(solucao));
         if (iterT % 2 > 0)
         {
            geraVizinhoAleatorio(s2, 3);
         }
         else
         {
            gerarVizinhoMudandoDeCarro(s2, 3);
         }

         calcFO(s2);
         variacao = s2.funObj - s1.funObj;
         if (variacao <= 0)
         {
            fflush(stdin);
            memcpy(&s1, &s2, sizeof(solucao));
            if (s2.funObj < Ms.funObj)
            {
               memcpy(&Ms, &s2, sizeof(solucao));
            }
         }
         else
         {
            fflush(stdin);
            x = (rand() % 1000) + 1;
            x /= 1000;
            if (x < pow(E, -(variacao / temperatura)))
            {
               memcpy(&s1, &s2, sizeof(solucao));
            }
         }
      }
      temperatura *= taxaResfriamento;
      iterT = 0;
   }

   memcpy(&s1, &Ms, sizeof(solucao));
}
