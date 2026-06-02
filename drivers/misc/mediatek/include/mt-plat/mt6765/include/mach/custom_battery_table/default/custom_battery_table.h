#ifndef __CUSTOM_BATTERY_TABLE__
#define __CUSTOM_BATTERY_TABLE__

#define BAT_NTC_68 1
#define RBAT_PULL_UP_R             62000
#define RBAT_PULL_UP_VOLT          1800
#define BIF_NTC_R 16000

/*
 * NTC data sheet value has low accuracy in low voltage
 * This value is tuning value
 */
struct FUELGAUGE_TEMPERATURE Fg_Temperature_Table[21] = {
		{-20, 738931},
		{-15, 547472},
		{-10, 409600},
		{-5, 309299},
		{0, 235622},
		{5, 181001},
		{10, 140153},
		{15, 109349},
		{20, 85934},
		{25, 68000},
		{30, 54165},
		{35, 43418},
		{40, 35014},
		{45, 28400},
		{50, 23164},
		{55, 18994},
		{60, 15655},
		{65, 12967},
		{70, 10791},
		{75, 9021},
		{80, 7574},
};
#endif
