#include <string.h>

#include "config.h"

const char program_name[] = "ffmulti";
const int program_birth_year = 2016;

int main_ffmpeg(int argc, char **argv);
int main_ffprobe(int argc, char **argv);
int main_ffserver(int argc, char **argv);
int main_ffplay(int argc, char **argv);

void show_help_default_ffmpeg(const char *opt, const char *arg);
void show_help_default_ffserver(const char *opt, const char *arg);
void show_help_default_ffprobe(const char *opt, const char *arg);
void show_help_default_ffplay(const char *opt, const char *arg);
void show_help_default(const char *opt, const char *arg);

static char tool = '\0';

void show_help_default(const char *opt, const char *arg)
{
	switch(tool) {
#if CONFIG_FFMPEG == 1
		case 'm': show_help_default_ffmpeg  (opt, arg); break;
#endif
#if CONFIG_FFSERVER == 1
		case 's': show_help_default_ffserver(opt, arg); break;
#endif
#if CONFIG_FFPROBE == 1
		case 'r': show_help_default_ffprobe (opt, arg); break;
#endif
#if CONFIG_FFPLAY == 1
		case 'l': show_help_default_ffplay  (opt, arg); break;
#endif
	}
}

int main(int argc, char **argv) {
	if (argc == 0 || !argv|| !argv[0]) return 127;
	if (0) {
#if CONFIG_FFMPEG == 1
	} else if (strstr(argv[0], "ffmpeg")) {
		tool='m';
		return main_ffmpeg(argc, argv);
#endif
#if CONFIG_FFPLAY == 1
	} else if (strstr(argv[0], "ffplay")) {
		tool='l';
		return main_ffplay(argc, argv);
#endif
#if CONFIG_FFPROBE == 1
	} else if (strstr(argv[0], "ffprobe")) {
		tool='r';
		return main_ffprobe(argc, argv);
#endif
#if CONFIG_FFSERVER == 1
	} else if (strstr(argv[0], "ffserver")) {
		tool='s';
		return main_ffserver(argc, argv);
#endif
	} else {
		return main(argc-1, argv+1);
	}
}
