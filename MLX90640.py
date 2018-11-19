from smbus2 import SMBus, i2c_msg
import math

class MLX90640:
	
	def __init__(address=0x33):    #default 0x33
		self.address = address
		self.bus = SMBus(1)
		self.addr = address
		self.gain = self.getGain()
		self.VDD0 = 3.3
		self.DV = self.getVDD()		
		self.VDD = self.DV+self.VDD0
		self.Ta0 = 25
		self.Ta = self.getTa()
		self.emissivity = 1
		self.TGC = self.getTGC()
		self.chessNotIL = 1
		self.KsTa = self.getKsTa()
		self.KsTo1, self.KsTo2, self.KsTo3, self.KsTo4 = self.getKsTo()
		self.step, self.CT3, self.CT4 = self.getCorners()
		self.CT1 = 40
		self.CT2 = 0		
		self.alphaCorrR1 = 1/float(1+ self.KsTo1*(0-(-40)))
		self.alphaCorrR2 = 1
		self.alphaCorrR3 = 1 + self.KsTo2*(self.CT3-0)
		self.alphaCorrR4 = self.alphaCorrR3*(1+self.KsTo3*(self.CT4-self.CT3))
		
	def getRegs(self,reg,num):
		write = i2c_msg.write(self.addr,[reg>>8,reg&0xFF])
		read = i2c_msg.read(self.address,num)
		self.bus.i2c_rdwr(write, read)
		return list(read)
			
	def getRegf(self,reg):
		write = i2c_msg.write(self.addr,[reg>>8,reg&0xFF])
		read = i2c_msg.read(self.address,2)
		self.bus.i2c_rdwr(write, read)
		result = list(read)
		return (result[0]<<8)+result[1]

	def root4(self,num):
		return math.sqrt(math.sqrt(num))
		
	def getTGC(self):
		TGC = self.getRegf(0x243C) & 0x00FF
		if TGC > 127:
			TGC = TGC - 256
		
		return TGC
		
	def getVDD(self):
		Kvdd = (self.getRegf(0x2433) & 0xFF00)/256
		if Kvdd > 127:
			Kvdd = Kvdd -256
		Kvdd = Kvdd*32
		
		Vdd25 = self.getRegf(0x2433) & 0x00FF
		Vdd25 = (Vdd25-256)*32 - 8192
		
		RAM = self.getRegf(0x072A)
		if RAM > 32767:
			RAM = RAM - 65536

		DV = (RAM - Vdd25)/float(Kvdd)	
		
		return DV

	def getTa(self):
		KVptat = (self.getRegf(0x2432) & 0xFC00)/1024
		if KVptat > 31:
			KVptat = KVptat - 62
		
		KVptat = KVptat/4096.0
		
		KTptat = self.getRegf(0x2432) & 0x03FF
		if KTptat > 511:
			KTptat = KTptat - 1022
		
		KTptat = KTptat/8.0
		
		Vptat25 = self.getRegf(0x2431)
		if Vptat25 > 32767:
			Vptat25 = Vptat25 - 65536
		Vptat = self.getRegf(0x0720)
		if Vptat > 32767:
			Vptat = Vptat - 65536
		Vbe = self.getRegf(0x0700)
		if Vbe > 32767:
			Vbe = Vbe - 65536
		AlphaptatEE = (self.getRegf(0x2410) & 0xF000)/4096
		Alphaptat = (AlphaptatEE/4)+8
		Vptatart = (Vptat/float(Vptat * Alphaptat + Vbe))*262144
		
		Ta = ((Vptatart/float(1+KVptat*self.DV)-Vptat25)/float(KTptat))+self.Ta0
		return Ta

	def getGain(self):
		GAIN = self.getRegf(0x2430)
		if GAIN > 32767:
			GAIN = GAIN - 65536
		RAM = self.getRegf(0x070A)
		if RAM > 32767:
			RAM = RAM - 65536
		return GAIN/float(RAM)
				
	def pixnum(self,i,j):
		return (i-1)*32 + j

	def patternChess(self,i,j):
		pixnum = self.pixnum(i,j)
		a = (pixnum-1)/32
		b = int((pixnum-1)/32)/2
		return int(a) - int(b)*2
		
	def getKsTa(self):
		KsTaEE = (self.getRegf(0x243C) & 0xFF00) >> 256
		if KsTaEE > 127:
			KsTaEE = KsTaEE -256
		
		KsTa = KsTaEE/8192.0
		return KsTa
	
	def getKsTo(self):
		EE1 = self.getRegf(0x243D)
		EE2 = self.getRegf(0x243E)
		KsTo1 = EE1 & 0x00FF
		KsTo3 = EE2 & 0x00FF
		KsTo2 = (EE1 & 0xFF00) >> 8
		KsTo4 = (EE2 & 0xFF00) >> 8

		if KsTo1 > 127:
			KsTo1 = KsTo1 -256
		if KsTo2 > 127:
			KsTo2 = KsTo2 -256
		if KsTo3 > 127:
			KsTo3 = KsTo3 -256
		if KsTo4 > 127:
			KsTo4 = KsTo4 -256
		
		KsToScale = (self.getRegf(0x243F) & 0x000F)+8
		KsTo1 = KsTo1/float(pow(2,KsToScale))
		KsTo2 = KsTo2/float(pow(2,KsToScale))
		KsTo3 = KsTo3/float(pow(2,KsToScale))
		KsTo4 = KsTo4/float(pow(2,KsToScale))
		return KsTo1, KsTo2, KsTo3, KsTo4
		
	def getCorners(self):
		EE = self.getRegf(0x243F)
		step = ((EE & 0x3000)>>12)*10
		CT3 = ((EE & 0x00f0)>>4)*step
		CT4 = ((EE & 0x0f00)>>8)*(step+CT3)
		return step, CT3, CT4
		
	def getPixData(self,i,j):
		Offsetavg = self.getRegf(0x2411)
		if Offsetavg > 32676:
			Offsetavg = Offsetavg-65536

		scaleVal = self.getRegf(0x2410)
		OCCscaleRow = (scaleVal&0x0f00)/256
		OCCscaleCol = (scaleVal&0x00F0)/16
		OCCscaleRem = (scaleVal&0x000F)
		rowAdd = 0x2412 + ((i-1)/4)
		colAdd = 0x2418 + ((j-1)/4)
		rowMask = 0xF<<(4*((i-1)%4))
		colMask = 0xF<<(4*((j-1)%4))

		OffsetPixAdd = 0x243F+((i-1)*32)+j
		OffsetPixVal = self.getRegf(OffsetPixAdd)
		OffsetPix = (OffsetPixVal & 0xFC00)/1024
		if OffsetPix >31:
			OffsetPix = OffsetPix - 64

		OCCRow = (self.getRegf(rowAdd) & rowMask)>>(4*((i-1)%4))
		if OCCRow >7:
			OCCRow = OCCRow -16

		OCCCol = (self.getRegf(colAdd) & colMask)>>(4*((j-1)%4))
		if OCCCol > 7:
			OCCCol = OCCCol -16

		pixOffset = Offsetavg + OCCRow*pow(2,OCCscaleRow) + OCCCol*pow(2,OCCscaleCol) + OffsetPix*pow(2,OCCscaleRem)
		
		KtaEE = (OffsetPixVal & 0x000E)/2
		if KtaEE > 3:
			KtaEE = KtaEE - 7

		colEven = not (j%2)
		rowEven = not (i%2)
		rowOdd = not rowEven
		colOdd = not colEven
		KtaAvAddr = 0x2436 + (colEven)
		KtaAvMask = 0xFF00 >> (8*rowEven)
		
		KtaRC = (self.getRegf(KtaAvAddr) & KtaAvMask) >> 8* rowOdd
		if KtaRC > 127:
			KtaRC = KtaAvRC - 256
		
		KtaScale1 = ((self.getRegf(0x2438) & 0x00F0) >>4)+8
		KtaScale2 = (self.getRegf(0x2438) & 0x000F)

		Kta = (KtaRC+(KtaEE<<KtaScale2))/float(pow(2,KtaScale1))
		
		shiftNum = (rowOdd*4)+(colOdd*8)
		KvMask = 0x000F << shiftNum
		Kv = (self.getRegf(0x2434) & KvMask) >> shiftNum
		if Kv > 7:
			Kv = Kv-16
		
		KvScale = (self.getRegf(0x2438) & 0x0F00)>>8
		
		Kv = Kv/float(KvScale)
		
		RAMaddr = 0x400+((i-1)*32)+ j-1
		RAM = self.getRegf(RAMaddr)
		if RAM > 32767:
			RAM = RAM - 65536
		pixGain = RAM*self.gain
		pixOs = pixGain - pixOffset*(1+Kta*(self.Ta - self.Ta0)*(1+Kv*(self.VDD - self.VDD0)))
		return pixOs

	def getCompensatedPixData(self,i,j):
		pixOs = self.getPixData(i,j)
		Kgain = ((self.gain -1)/10)+1
		
		pixGainCPSP0 = self.getRegf(0x0708)		
		if pixGainCPSP0 > 32767:
			pixGainCPSP0 = pixGainCPSP0 - 65482
		
		pixGainCPSP1 = self.getRegf(0x0728)
		if pixGainCPSP1 > 32767:
			pixGainCPSP1 = pixGainCPSP1 - 65482
		
		pixGainCPSP0 = pixGainCPSP0*Kgain
		pixGainCPSP1 = pixGainCPSP1*Kgain
		
		OffCPSP0 = self.getRegf(0x243A) & 0x03FF
		if OffCPSP0 > 511:
			OffCPSP0 = OffCPSP0-1024
		
		OffCPSP1d = (self.getRegf(0x243A) &0xFC00)>>10
		if OffCPSP1d > 31:
			OffCPSP1d = OffCPSP1d-64
		
		OffCPSP1 = OffCPSP1d + OffCPSP0
		
		KvtaCPEEVal = self.getRegf(0x243B)
		KvtaScaleVal = self.getRegf(0x2438)
		
		KtaScale1 = ((KvtaScaleVal & 0x00F0)>>4)+8
		KvScale = (KvtaScaleVal & 0x0F00)>>8
		
		KtaCPEE = KvtaCPEEVal & 0x00FF
		if KtaCPEE > 127:
			KtaCPEE = KtaCPEE -256
		
		KvCPEE = (KvtaCPEEVal & 0xFF00)>>8
		
		KtaCP = KtaCPEE/float(pow(2,KtaScale1))
		KvCP = KvCPEE/float(pow(2,KvScale))
		
		b = (1+KtaCP*(self.Ta - self.Ta0))*(1+ KvCP*(self.VDD - self.VDD0))
		pixOSCPSP0 = pixGainCPSP0 - OffCPSP0*b
		pixOSCPSP1 = pixGainCPSP1 - OffCPSP1*b
		
		if self.chessNotIL:
			pattern = self.patternChess(i,j)
		else:
			pattern = self.patternChess(i,j)
		
		VIREmcomp = pixOs/self.emissivity
		VIRcomp = VIREmcomp - self.TGC*((1-pattern)*pixOSCPSP0 + pattern*pixOSCPSP1)
		###########TESTED TO HERE
		
		reg2439val = self.getRegf(0x2439)
		reg2420val = self.getRegf(0x2420)
		alphaScaleCP = ((reg2420val & 0xF000)>>12) + 27
		CPP1P0ratio = (reg2439val & 0xFC00)>>10
		if CPP1P0ratio >31:
			CPP1P0ratio = CPP1P0ratio -64
		
		alphaRef = self.getRegf(0x2421)
		alphaScale = ((reg2420val & 0xF000)>>12) + 30
		
		rowAdd = 0x2422 + ((i-1)/4)
		colAdd = 0x2428 + ((j-1)/4)
		rowMask = 0xF<<(4*((i-1)%4))
		colMask = 0xF<<(4*((j-1)%4))		

		ACCRow = (self.getRegf(rowAdd) & rowMask)>>(4*((i-1)%4))
		if ACCRow >7:
			ACCRow = ACCRow -16

		ACCCol = (self.getRegf(colAdd) & colMask)>>(4*((j-1)%4))
		if ACCCol > 7:
			ACCCol = ACCCol -16
		
		ACCScaleRow = (reg2420val & 0x0F00)>>8
		ACCScaleCol = (reg2420val & 0x00F0)>>4
		ACCScaleRem = (reg2420val & 0x000F)
		
		alphaPixel = (self.getRegf(0x241f+self.pixnum(i,j))&0x03f0)>>4
		
		alpha = (alphaRef+(ACCRow<<ACCScaleRow)+(ACCCol<<ACCScaleCol)+(alphaPixel<<ACCScaleRem))/float(pow(2,alphaScale))
		
		alphaCPSP0 = (reg2439val & 0x03ff)/float(pow(2,alphaScaleCP))
		alphaCPSP1 = alphaCPSP0*(1+CPP1P0ratio/128.0)
		alphacomp= alpha - self.TGC*((1-pattern)*alphaCPSP0 + pattern*alphaCPSP1)*(1+self.KsTa*(self.Ta-self.Ta0))

		Tak4 = pow(self.Ta + 273.15,4)
		Trk4 = pow(self.Ta-8 + 273.15,4)
		Tar = Trk4-(Trk4-Tak4)/self.emissivity
		Sx = self.KsTo2*self.root4(pow(alphacomp,3)*VIRcomp + pow(alphacomp,4)*Tar)
		return self.root4((VIRcomp/(alphacomp*(1-self.KsTo2*273.15)+Sx))+Tar) - 273.15
