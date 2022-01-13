#include <sys/socket.h>
#include <pthread.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include "client.h"

void *in(void *sockfd)
{
	char buff[MAX];
	int n;
	for (;;) {
		bzero(buff, sizeof(buff));
		read((int *)sockfd, buff, sizeof(buff));
		if ((strncmp(buff, "", 4)) == 0) {
			break;
		}
	}
}

void *out(void *sockfd)
{
	char buff[MAX];
	int n;
	for (;;) {
		bzero(buff, sizeof(buff));
		printf("Enter the string : ");
		n = 0;
		while ((buff[n++] = getchar()) != '\n')
			;
		write((int *)sockfd, buff, sizeof(buff));
		if ((strncmp(buff, "exit", 4)) == 0) {
			printf("Client Exit...\n");
			break;
		}
	}
}

char *init()
{
	int sockfd, connfd;
	struct sockaddr_in servaddr, cli;

	in_buff = 

	// socket create and varification
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		printf("socket creation failed...\n");
		exit(0);
	}
	else
		printf("Socket successfully created..\n");
	bzero(&servaddr, sizeof(servaddr));

	// assign IP, PORT
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr("192.168.0.11");
	servaddr.sin_port = htons(PORT);

	// connect the client socket to server socket
	if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) {
		printf("connection with the server failed...\n");
		exit(0);
	}
	else
		printf("connected to the server..\n");

	pthread_t thread_in;
    printf("Running in...\n");
	pthread_create(&thread_in, NULL, in, sockfd);

	pthread_t thread_out;
    printf("Running out...\n");
	pthread_create(&thread_out, NULL, out, sockfd);
    
    pthread_join(thread_in, NULL);
	pthread_join(thread_out, NULL);

	return in_buff;
}
