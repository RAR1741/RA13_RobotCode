#include "FRCLED.h"

using namespace std;

FRCLED::FRCLED(int spike1, int spike2,int spike3)
{
	s1 = new Relay(spike1);
	s2 = new Relay(spike2);
	s3 = new Relay(spike3);
	time = 0;
	state = 0;
}

void FRCLED::pattern1()
{
	switch(state)
	{
	case 1:
		if(time == DELAY)
			state = 2;
		s1->Set(triangle);
		s2->Set(circle);
		s3->Set(square);
		time++;
		break;
	case 2:
		if(time == DELAY*2)
		{
			state = 1;
			time = 0;
		}
		s1->Set(off);
		s2->Set(off);
		s3->Set(off);
		time++;
		break;
	}
}

void FRCLED::pattern2()
{
	switch(state)
		{
		case 1:
			if(time == DELAY)
				state = 2;
			s1->Set(off);
			s2->Set(off);
			s3->Set(triangle);
			time++;
			break;
		case 2:
			if(time == DELAY*2)
				state = 3;
			s1->Set(off);
			s2->Set(triangle);
			s3->Set(circle);
			time++;
			break;
		case 3:
			if(time == DELAY*3)
				state = 4;
			s1->Set(triangle);
			s2->Set(circle);
			s3->Set(square);
			break;
		case 4:
			if(time == DELAY*4)
				state = 5;
			s1->Set(circle);
			s2->Set(square);
			s3->Set(off);
			break;
		case 5:
			if(time == DELAY*5)
				state = 6;
			s1->Set(square);
			s2->Set(off);
			s3->Set(off);
		case 6:
			if(time == DELAY*6)
			{
				state = 1;
				time = 0;
			}
			s1->Set(off);
			s2->Set(off);
			s3->Set(off);
		}
}

void FRCLED::pattern3()
{
	switch(state)
		{
		case 1:
			if(time == DELAY)
				state = 2;
			s1->Set(triangle);
			s2->Set(triangle);
			s3->Set(triangle);
			time++;
			break;
		case 2:
			if(time == DELAY*2)
				state = 3;
			s1->Set(circle);
			s2->Set(circle);
			s3->Set(circle);
			time++;
			break;
		case 3:
			if(time == DELAY*3)
			{
				state = 1;
				time = 0;
			}
			s1->Set(square);
			s2->Set(square);
			s3->Set(square);
			break;
		}
}

void FRCLED::pattern4()
{
	switch(state)
	{
	case 1:
		if(time == DELAY)
			state = 2;
		s1->Set(triangle);
		s2->Set(off);
		s3->Set(off);
		time++;
		break;
	case 2:
		if(time == DELAY*2)
			state = 3;
		s1->Set(triangle);
		s2->Set(circle);
		s3->Set(off);
		time++;
		break;
	case 3:
		if(time == DELAY*3)
			state = 4;
		s1->Set(triangle);
		s2->Set(circle);
		s3->Set(square);
		break;
	case 4:
		if(time == DELAY*4)
		{
			state = 1;
			time = 0;
		}
		s1->Set(off);
		s2->Set(off);
		s3->Set(off);
		break;
	}
}

void FRCLED::resetTime()
{
	time = 0;
}
