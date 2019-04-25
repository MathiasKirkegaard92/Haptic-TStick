#include "adsrTable.h"

const int adsr::tableLength = 512;
const float adsr::minScale = 1 / adsr::tableLength * 4;
const float adsr::adsrTable[adsr::tableLength] = {
	0,	0.00121311353838867,	0.00245565996591777,	0.00372835339131311,	0.00503192524919975,	0.00636712472046770,	0.00773471916283653,
	0.00913549455186662,	0.0105702559326702,	0.0120398278825820,	0.0135450549850552,	0.0150868023150557,	0.0166659559362321,	0.0182834234101494,
	0.0199401343178769,	0.0216370407942317,	0.0233751180749834,	0.0251553650573360,	0.0269788048740079,	0.0288464854812407,	0.0307594802610738,
	0.0327188886382326,	0.0347258367119827,	0.0367814779033150,	0.0388869936178327,	0.0410435939247219,	0.0432525182521947,	0.0455150360998062,
	0.0478324477680533,	0.0502060851056759,	0.0526373122750883,	0.0551275265363830,	0.0576781590503550,	0.0602906757010101,	0.0629665779380287,
	0.0657074036396699,	0.0685147279966116,	0.0713901644172344,	0.0743353654548698,	0.0773520237575459,	0.0804418730407754,	0.0836066890839462,
	0.0868482907508866,	0.0901685410351916,	0.0935693481309111,	0.0970526665292160,	0.100620498141671,	0.104274893450761,	0.108017952688333,
	0.111851827042626,	0.115778719894593,	0.119800888084211,	0.123920643207518,	0.128140352945124,	0.132462442422946,	0.136889395605960,
	0.141423756725778,	0.146068131742843,	0.150825189844119,	0.155697664977107,	0.160688357421080,	0.165800135396448,	0.171035936713158,
	0.176398770459101,	0.181891718729471,	0.187517938398090,	0.193280662931708,	0.199183204248320,	0.205228954620574,	0.211421388625354,
	0.217764065140670,	0.224260629390992,	0.230914815042217,	0.237730446347450,	0.244711440344864,	0.251861809108871,	0.259185662055921,
	0.266687208306235,	0.274370759102851,	0.282240730289352,	0.290301644847715,	0.298558135497729,	0.307014947359485,	0.315676940680464,
	0.324549093628784,	0.333636505154230,	0.342944397918684,	0.352478121297666,	0.362243154454695,	0.372245109490236,	0.382489734667056,
	0.392982917713831,	0.403730689208902,	0.414739226046137,	0.426014854984873,	0.437564056285999,	0.449393467436243,	0.461509886962839,
	0.473920278340724,	0.486631773994554,	0.499651679397800,	0.512987477271311,	0.526646831883733,	0.540637593456275,	0.554967802674337,
	0.569645695308607,	0.584679706948269,	0.600078477849059,	0.615850857898935,	0.632005911704233,	0.648552923799223,	0.665501403982060,
	0.682861092780194,	0.700641967048394,	0.718854245702576,	0.737508395592764,	0.756615137518530,	0.776185452390389,	0.796230587540681,
	0.816762063187564,	0.837791679055855,	0.859331521158484,	0.881393968742502,	0.903991701403603,	0.927137706373267,	0.950845285982696,
	0.975128065307858,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	0.937456498273903,	0.886052663336752,	0.842413864766291,	0.804499101325208,
	0.770979391392951,	0.740940818769470,	0.713727872255709,	0.688854364912539,	0.665949712154884,	0.644724958484602,	0.624950434652479,
	0.606440572028042,	0.589043287806453,	0.572632383409929,	0.557101984577306,	0.542362398509608,	0.528336975613354,	0.514959696988562,
	0.502173295133063,	0.489927772407308,	0.478179220324058,	0.466888869215772,	0.456022316358558,	0.445548893790675,	0.435441146544016,
	0.425674398926617,	0.416226391605972,	0.407076976061345,	0.398207855855273,	0.389602366370676,	0.381245286348774,	0.373122675872542,
	0.365221736463895,	0.357530689768665,	0.350038671942382,	0.342735641359822,	0.335612297680843,	0.328660010635827,	0.321870757162829,
	0.315237065747970,	0.308751967000756,	0.302408949644538,	0.296201921225389,	0.290125172945069,	0.284173348109289,	0.278341413754253,
	0.272624635074847,	0.267018552328959,	0.261518959935633,	0.256121887521670,	0.250823582702692,	0.245620495411619,	0.240509263610622,
	0.235486700242543,	0.230549781294945,	0.225695634864853,	0.220921531125172,	0.216224873105001,	0.211603188205894,	0.207054120384665,
	0.202575422940878,	0.198164951853737,	0.193820659618919,	0.189540589540986,	0.185322870441576,	0.181165711747520,	0.177067398926642,
	0.173026289242105,	0.169040807798993,	0.165109443859325,	0.161230747403920,	0.157403325921526,	0.153625841407427,	0.149897007555338,
	0.146215587127842,	0.142580389491911,	0.138990268307239,	0.135444119356144,	0.131940878504774,	0.128479519786183,	0.125059053596650,
	0.121678524997303,	0.118337012113749,	0.115033624627016,	0.111767502349606,	0.108537813880980,	0.105343755337207,	0.102184549149923,
	0.0990594429301128,	0.0959677083925605,	0.0929086403371215,	0.0898815556832519,	0.0868857925544887,	0.0839207094098122,	0.0809856842190383,
	0.0780801136795869,	0.0752034124721661,	0.0723550125530668,	0.0695343624809345,	0.0667409267760182,	0.0639741853100360,	0.0612336327249181,
	0.0585187778788021,	0.0558291433177617,	0.0531642647718489,	0.0505236906741161,	0.0479069817013762,	0.0453137103355281,	0.0427434604443547,
	0.0401958268807663,	0.0376704150995194,	0.0351668407905102,	0.0326847295277862,	0.0302237164334744,	0.0277834458558742,	0.0253635710610027,
	0.0229637539369225,	0.0205836647102253,	0.0182229816740713,	0.0158813909272277,	0.0135585861235772,	0.0112542682315911,	0.00896814530330248,
	0.00669993225232579,	0.00444935064050744,	0.00221612847280195,	0
};

adsr::Env::Env(float attack = 1, float decay = 1, float sustain = 1, float release = 1): offset(0), tableIdx(0), gain(1), sustainGain(1) {
	setDur(attack, decay, sustain, release);
};

void adsr::Env::setDur(float attack = 1, float decay = 1, float sustain = 1, float release = 1) {
	dur[0] = max(min(attack, 1.0f), adsr::minScale); dur[1] = max(min(decay, 1.0f), adsr::minScale);
	dur[2] = max(min(sustain, 1.0f), adsr::minScale); dur[3] = max(min(release, 1.0f), adsr::minScale);
};

// Offset parameter in range. [0;1]
void adsr::Env::setOffset(float the_offset) {
	// Serial.printf("before :");
	// Serial.println(tableIdx);
	tableIdx -= offset;
	offset = tableLength * the_offset;
	tableIdx += offset;
	tableIdx = wrap(tableIdx);
	// Serial.print("the_offset :");
	// Serial.println(the_offset);
	// Serial.println(" --- ");
};

void adsr::Env::setGains(float the_gain, float the_sustainGain) {
	gain = max(min(the_gain, 1.0f), 0.0f); sustainGain = max(min(the_sustainGain, 1.0f), 0.0f);
}


float adsr::Env::wrap(float the_tableIdx) {
	// the_tableIdx = max(the_tableIdx, 0.0f);
	while (the_tableIdx < 0) the_tableIdx += tableLength;
	while (the_tableIdx > (tableLength - 1)) the_tableIdx -= tableLength;
	return the_tableIdx;
}
//  Clips index to be within range of section.
float adsr::Env::clipSection(float the_tableIdx, int section) {
	return section == 3 ? wrap(tableIdx) : min(max(the_tableIdx, 0.0f), tableLength / 4.0f * (section + 1.0f));
}

float adsr::Env::clip(float the_tableIdx) {
	return min(max(the_tableIdx, 0.0f), (float) tableLength);
};


float adsr::Env::update(float rate) {
	// Apply scaling on idx according to dur parameter: 1 = attack, 2 = decay, 3 = sustain, 4 = release
	int section = max(int(std::floor(tableIdx / tableLength * 4)), 0); 	// Find out which section (dur[?]) we are in

	tableIdx += rate / dur[section];
	tableIdx = clipSection(tableIdx, section); 								// clip tableIdx to current section
	int tableIdxInt = std::floor(tableIdx);					 // Calculate lo and high index. OBS. always difference of 1 ??
	int frac = tableIdx - tableIdxInt;
	return  (section == 2 ? sustainGain : gain) * ( (1 - frac) * adsrTable[tableIdxInt] + frac * adsrTable[tableIdxInt + 1]); // return linearly interpolated table value
};

float adsr::Env::readTable(float phase, boolean normalized) {
	float idx = normalized ? phase2Idx(phase) : phase;
	int idxInt = std::floor(idx);
	float frac = idx - idxInt;
	return (1 - frac) * adsrTable[idxInt] + frac * adsrTable[idxInt + 1];
}

float adsr::Env::phase2Idx(float phase) {
	static float lastPhase;
	// Hysteresis for use with expressive parameters.
	// Exzponential scaling upwards, logarithmnic downwards.
	// int slopeSgn = phase > lastPhase ? 1 : 0;
	// lastPhase = phase;
	float idx = phase * tableLength;// + tableLength / 2.0f * slopeSgn;
	int section = max(int(std::floor(idx / tableLength * 4)), 0); 	// Find out which section (dur[?]) we are in
	idx += 1.0f / dur[section];
	return clipSection(idx, section);
}
