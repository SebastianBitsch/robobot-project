#include <iostream>
#include <unistd.h>
#include <ctype.h>
#include <string.h>

#include "userverport.h"

int main(int argc, char **argv) {
    
//   std::cout << "Hello, world!" << std::endl;
  printf("# socket server test\n");
    
  UServerPort * server = new UServerPort();
  const int SL = 100;
  char c[SL] = " ";
  char * pc = NULL;
  UServerInMsg * msg;
  int cnt = 1;
  std::cout << "Server object created" << std::endl;
  server->setPort(24000);
  server->setVerbose(false);
  server->setAllowConnections(true);
  server->start();
  usleep(100000);
  printf("--- Press ^C to quit\n");
  while (c[0] != 'q')
  {
    printf(">>");
    pc = fgets(c, SL, stdin);
    if (pc == NULL)
      break;
    switch (c[0])
    {
      case 'p':
        server->print("Server ");
        break;
      case 'n':
        cnt = 1;
        sscanf(&c[1], "%d", &cnt);
        for (int n = 0; n < cnt; n++)
        {
          if (server->getRxQueue()->isEmpty())
          {
            printf("Queue is empty\n");
            break;
          }
          msg = server->getRxQueue()->skipToNextMessage(true);
          msg->print("got");
        }
        break;
      case 's':
        pc = &c[1];
        cnt = 0;
        if (isdigit(*pc))
        {
          cnt = strtol(pc, &pc, 10);
        }
        if (*pc >= ' ')
        {
          UServerClient * client;
          printf(" ---- getting client %d\n", cnt);
          client = server->getClient(cnt);
          if (client != NULL)
          {
            if (client->isActive())
            {
              printf(" ---- client is active %d\n", cnt);
              client->blockSend(pc, strlen(pc), 100);
            }
          }
          else
              printf("# no such client (%d)\n", cnt);
        }
        break;
      case 'k':
        server->getRxQueue()->print("# RX queue:");
        break;
      default:
        printf("Press 'q'       (quit), \n");
        printf("Press 'p'       (print server) or \n");
        printf("Press 'k'       (print queue) or \n");
        printf("Press 'n <cnt>' (get cnt messages from queue)\n");
        printf("Press 'sNmsg'   (send msg to client N\n");
        break;
    }
  }    
  printf("-- terminated --\n");  
  return 0;
}
