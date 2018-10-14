#/*
 * Third Year Project
 * Matt Kimber
 *
 * Sound routines
 */

#ifndef CONFIG
#include "config.h"
#endif


#define MAX_SAMPLES 4

/* SDL is used for sound output */
#ifdef SOUND
#include <SDL/SDL_mixer.h>
#include <math.h>

Mix_Music *music;
Mix_Chunk *audio_sample[MAX_SAMPLES];
int sample_active[MAX_SAMPLES];
Uint32 audio_length[MAX_SAMPLES];
#endif

struct VALVE {
  float open_time;
  float open_duration;
  float pressure;
};

struct VALVE valve[4];

int cylinders;
float cylinder_offset[12];

float skid_volume = 0.0;

int mus_volume = 48;

int sound_is_working;
int music_playing;
float pitch_multiplier=1.0;

void Snd_LoadMP3(char *filename)
{
  #ifdef SOUND
  if(sound_is_working) {
    music = Mix_LoadMUS(filename);
    if(!music) {
      printf("Could not load MP3 track %s - error %s\n.", 
	     filename, Mix_GetError());
      return;
    }

    /* Start the music playing */
    if (Mix_PlayMusic(music, -1) == -1 ) {
      printf("Could not play MP3 track %s - error %s\n.", 
	     filename, Mix_GetError());
      return;
    }
  
    Mix_VolumeMusic(mus_volume);
    music_playing = 1;
  }
  #endif
}

void Snd_MusicVolumeDown()
{
#ifdef SOUND
  if(mus_volume > 16) {
    mus_volume -= 16;
    Mix_VolumeMusic(mus_volume);
  }
#endif
}

void Snd_MusicVolumeUp()
{
#ifdef SOUND
  if(mus_volume < 192) {
    mus_volume += 16;
    Mix_VolumeMusic(mus_volume);
  }
#endif
}

void Snd_PlayPauseMusic()
{
#ifdef SOUND
  /* Start the music up again if it was paused */

  if(music_playing) {
    music_playing = 0;
    Mix_PauseMusic();
  } else {
    music_playing = 1;
    Mix_ResumeMusic();
  }
#endif
}

void Snd_LoadSound(char *filename, int sample_num)
{
  #ifdef SOUND
  /* Don't load if given an invalid sample number */
  if ((sample_num > (MAX_SAMPLES - 1)) || (!sound_is_working)) return;

  audio_sample[sample_num] = Mix_LoadWAV(filename);
  if(!audio_sample[sample_num]) {
    printf("Could not load sound sample: %s\n", Mix_GetError());
  } else {
    sample_active[sample_num] = 1;
  }
  #endif
}


void SetCylinders(int c)
{
  cylinders = c;
}

void SetCylOffset(int c, float f)
{
  cylinder_offset[c] = f;
}

void SetValveOpenPos(int v, float f)
{
  valve[v].open_time = f;
}

void SetValveOpenDuration(int v, float f)
{
  valve[v].open_duration = f;
}

void SetValvePressureDrop(int v, float f)
{
  valve[v].pressure = f;
}

void SetSkidVolume(float f)
{
  skid_volume = f;
  if (skid_volume > 0.05) skid_volume = 0.05;
}

int CylinderSound(float t)
{

#ifdef SOUND
  float final_multiplier = 32000.0;
  int i;

  while(t > M_PI) {
    t -= M_PI;
  }

  /* Valve openings */
  for(i=0;i<4;i++) {
    if ((t > valve[i].open_time) && 
	(t < valve[i].open_time + valve[i].open_duration)) {
      final_multiplier = final_multiplier * valve[i].pressure;
    }
  }

  return (int)(sin(t) * final_multiplier);
#endif
}

void PitchShift(int channel, void *stream, int len, void *udata)
{
#ifdef SOUND
  int i, j;
  static float f;

  Uint16 *test;

  test = (Uint16*)stream;

  for(i=0;i<=len/2;i++) {
    f += pitch_multiplier / (600000.0 * (float)cylinders);

    test[i] = (int)((float)test[i] * skid_volume);

    for(j=0;j<cylinders;j++) {
      test[i] += ( CylinderSound(f+cylinder_offset[j]) / 4.0 );
    }

  }

  while (f > M_PI) f -= M_PI;
#endif
}

void Snd_SetPitch(float p)
{
  /* Set the pitch multiplier */
  #ifdef SOUND
  pitch_multiplier = p;
  #endif
}

void Snd_FreeSounds()
{
  #ifdef SOUND
  int i;
  for(i=0;i<MAX_SAMPLES;i++) {
    if (sample_active[i]) Mix_FreeChunk(audio_sample[i]);
  }

  /* Close the music */
  Mix_HaltMusic();
  #endif
}

void Snd_CloseAudio()
{
  #ifdef SOUND

  if(sound_is_working) {
    Mix_ExpireChannel(-1, 0);
    Mix_UnregisterAllEffects(-1);

    Mix_CloseAudio();
  }

  #endif
}

void Snd_AddEffect(int channel,int effect)
{
  #ifdef SOUND
  if(sound_is_working) {
    switch(effect) {
    case 1:
      /* Pitch shift effect */
      Mix_RegisterEffect(channel, PitchShift, NULL, NULL);
      break;
    }
  }
  #endif
}

void Snd_StartSampleLoop(int sample_num, int channel)
{
  #ifdef SOUND
  if(sound_is_working) {
    if(sample_num > (MAX_SAMPLES - 1)) return;
    if(sample_active[sample_num]) {
      Mix_PlayChannel(channel, audio_sample[sample_num], -1);
    }
  }
  #endif
}

void Snd_PlaySound(int sample_num, int channel)
{
  #ifdef SOUND
  if(sample_num > (MAX_SAMPLES - 1)) return;
  if(sample_active[sample_num]) {
    Mix_PlayChannel(channel, audio_sample[sample_num], 0);
  }
  #endif
}

void Snd_SetVolume(int channel, int volume)
{
  #ifdef SOUND
  Mix_Volume(channel, volume);
  #endif
}

void Snd_StartAudio()
{
  #ifdef SOUND
  int i;

  /* Open the SDL_Mixer audio device */
  if (Mix_OpenAudio(44100,MIX_DEFAULT_FORMAT,2,512) == -1) {
    printf("Could not initialise audio: %s.\n", Mix_GetError());
    sound_is_working = 0;
  } else {
    sound_is_working = 1;
  }

  /* 4 mixing channels */
  if (sound_is_working) {
    Mix_AllocateChannels(4);


    for(i=0;i<MAX_SAMPLES;i++) {
      sample_active[i] = 0;
    }
  }
  #endif
}
