// mps_atcmd_test sample code with HTML output and timestamp
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/select.h>
#include <time.h>

#define BUFFER_SIZE 1024
#define COMMAND_FILE "/data/at_commands_list"             // Fixed command file path
#define LOG_FILE "/var/log/mps_atcmd_test_result.log"     // Log file path
#define HTML_FILE "/var/log/mps_atcmd_test_result.html"   // HTML report path
#define TATP_SOCKET_PATH "/var/run/m2mb_ssd_serv"         // Socket path
#define TIMEOUT_SEC 10                                    // Max wait time for response

// Initialize HTML file with header
void init_html() {
  FILE *html_fp = fopen(HTML_FILE, "w");
  if (html_fp) {
    fprintf(html_fp,
      "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>AT Command Test Result</title>"
      "<style>table{border-collapse:collapse;width:100%%;}th,td{border:1px solid #ccc;padding:8px;text-align:left;}th{background:#f2f2f2;} .ok{color:green;font-weight:bold;} .error{color:red;font-weight:bold;} .timeout{color:orange;font-weight:bold;}</style>"
      "</head><body><h2>AT Command Test Result</h2><table><tr><th>Timestamp</th><th>AT Command</th><th>AT Response</th><th>Status</th></tr>");
    fclose(html_fp);
  }
}

// Append a row to HTML file
void append_html(const char *timestamp, const char *cmd, const char *resp, const char *status_class, const char *status_text) {
  FILE *html_fp = fopen(HTML_FILE, "a");
  if (html_fp) {
    fprintf(html_fp, "<tr><td>%s</td><td>%s</td><td>%s</td><td class='%s'>%s</td></tr>\n",
            timestamp, cmd, resp, status_class, status_text);
    fclose(html_fp);
  }
}

// Finalize HTML file
void finalize_html() {
  FILE *html_fp = fopen(HTML_FILE, "a");
  if (html_fp) {
    fprintf(html_fp, "</table></body></html>");
    fclose(html_fp);
  }
}

int main() {
  int sock_fd;
  struct sockaddr_un addr;
  char at_rsp[BUFFER_SIZE];
  char tmp_rsp[BUFFER_SIZE];
  int bytes_read, tot_len;

  // Initialize HTML file
  init_html();

  // Create socket
  if ((sock_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
    perror("Failed to create socket");
    exit(EXIT_FAILURE);
  }

  memset(&addr, 0, sizeof(struct sockaddr_un));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, TATP_SOCKET_PATH, sizeof(addr.sun_path) - 1);

  // Connect to socket
  if (connect(sock_fd, (struct sockaddr*)&addr, sizeof(struct sockaddr_un)) == -1) {
    perror("Failed to connect socket");
    close(sock_fd);
    exit(EXIT_FAILURE);
  }

  // Open command file
  FILE *fp = fopen(COMMAND_FILE, "r");
  if (!fp) {
    perror("Failed to open command file");
    close(sock_fd);
    exit(EXIT_FAILURE);
  }

  // Open log file in append mode
  FILE *log_fp = fopen(LOG_FILE, "a");
  if (!log_fp) {
    perror("Failed to open log file");
    fclose(fp);
    close(sock_fd);
    exit(EXIT_FAILURE);
  }

  char line[256];
  while (fgets(line, sizeof(line), fp)) {
    // Remove newline characters
    line[strcspn(line, "\r\n")] = '\0';

    // Check for TEST_END to terminate
    if (strcmp(line, "TEST_END") == 0) {
      printf("TEST_END detected. Exiting...\n");
      break;
    }

    // Add carriage return for AT command
    char at_cmd[260];
    snprintf(at_cmd, sizeof(at_cmd), "%s\r", line);

    // Send AT command
    if (write(sock_fd, at_cmd, strlen(at_cmd)) == -1) {
      perror("Failed to write to socket");
      break;
    }

    printf("AT client sent AT command: %s\n", at_cmd);

    // Prepare for response
    tot_len = 0;
    memset(at_rsp, 0, sizeof(at_rsp));

    // Use select() for timeout
    fd_set read_fds;
    struct timeval timeout;
    timeout.tv_sec = TIMEOUT_SEC;
    timeout.tv_usec = 0;

    FD_ZERO(&read_fds);
    FD_SET(sock_fd, &read_fds);

    int ret = select(sock_fd + 1, &read_fds, NULL, NULL, &timeout);
    time_t now = time(NULL);
    char timestamp[64];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&now));

    if (ret == -1) {
      perror("select error");
      break;
    } else if (ret == 0) {
      // Timeout occurred
      printf("No response within %d seconds. Skipping...\n", TIMEOUT_SEC);
      fprintf(log_fp, "[TIMEOUT] CMD: %s | No response within %d sec\n", line, TIMEOUT_SEC);
      fflush(log_fp);
      append_html(timestamp, line, "No response", "timeout", "TIMEOUT");
      sleep(1);
      continue;
    }

    // If data is available, read response
    do {
      bytes_read = read(sock_fd, tmp_rsp, BUFFER_SIZE - 1);
      if (bytes_read > 0) {
        if (tot_len + bytes_read > BUFFER_SIZE - 1) {
          fprintf(stderr, "Buffer overflow\n");
          break;
        }

        memcpy(at_rsp + tot_len, tmp_rsp, bytes_read);
        tot_len += bytes_read;
        at_rsp[tot_len] = '\0';

        // Check for OK or ERROR in response
        if (strstr(at_rsp, "\r\nOK\r\n") || strstr(at_rsp, "\r\nERROR\r\n")) {
          printf("AT client received AT response: %s\n", at_rsp);

          // Write to log file with timestamp
          fprintf(log_fp, "[%s] CMD: %s | RESP: %s\n", timestamp, line, at_rsp);
          fflush(log_fp);

          // Determine status for HTML
          const char *status_class = strstr(at_rsp, "OK") ? "ok" : "error";
          const char *status_text = strstr(at_rsp, "OK") ? "OK" : "ERROR";
          append_html(timestamp, line, at_rsp, status_class, status_text);
          break;
        }
      } else {
        perror("Failed to read from socket");
        break;
      }
    } while (1);

    // Wait 1 second before next command
    sleep(1);
  }

  fclose(fp);
  fclose(log_fp);
  close(sock_fd);

  // Finalize HTML file
  finalize_html();

  return 0;
}
