#include "CFremenGrid.h"

using namespace std;

CFremenGrid::CFremenGrid(int dx,int dy,int dz)
{
	xDim = dx;
	yDim = dy;
	zDim = dz;
	signalLength = 0;
	numCells = xDim*yDim*zDim;
	order = 0;
	cellArray = (CFrelement**) malloc(numCells*sizeof(CFrelement*));
	for (int i=0;i<numCells;i++) cellArray[i] = new CFrelement();
	plan = new CFFTPlan();
}

CFremenGrid::~CFremenGrid()
{
	for (int i=0;i<numCells;i++) free(cellArray[i]);
	delete plan;
	free(cellArray);
}

void CFremenGrid::setName(const char* n)
{
	strcpy(name,n);
}

void CFremenGrid::setPose(float x, float y)
{
	positionX = x;
	positionY = y;
}

SGridErrors CFremenGrid::update(int order,int signalLengthi,bool evaluate)
{
	signalLength = signalLengthi;
	plan->prepare(signalLength);
	cout << "Signal length " << signalLength << " of " << cellArray[0]->getLength() << endl;
	SGridErrors e;
	float err = 0;
	int numDynamicCells = 0;
	int numNonempty = 0;
	float localError = 0;
	for (int i=0;i<numCells;i++){
		localError = cellArray[i]->update(order,plan,evaluate);
		if (localError >= 0){
			err += localError;
			numDynamicCells++;
		}
		if (localError > -2) numNonempty++;
	}
	e.all = err/numCells;
	e.dynamic = err/numDynamicCells;
	e.nonempty = err/numNonempty;
	return e;
}

int CFremenGrid::evaluatePrecision(SGridErrors e[],int maxOrder)
{
	plan->prepare(signalLength);
	int numDynamicCells = 0;
	int numNonempty = 0;
	float localError[maxOrder];
	float sumError[maxOrder];
	memset(sumError,0,sizeof(float)*maxOrder);
	for (int i=0;i<numCells;i++){
		cellArray[i]->evaluatePrecision(plan,localError,maxOrder);
		if (localError[0] >= 0) numDynamicCells++;
		if (localError[0] > -2) numNonempty++;
		for (int j=0;j<maxOrder;j++){
			if (localError[0] >= 0)	sumError[j] += localError[j];
		}
	}
	for (int j=0;j<maxOrder;j++){
		e[j].all = sumError[j]/numCells;
		e[j].dynamic = sumError[j]/numDynamicCells;
		e[j].nonempty = sumError[j]/numNonempty;
	}
}

void CFremenGrid::updateOne(int cellIndex, int order)
{
	plan->prepare(signalLength);
	cellArray[cellIndex]->update(order,plan);
}

void CFremenGrid::save(const char* filename,bool lossy,int forceOrder)
{
	FILE* f=fopen(filename,"w");
	unsigned char nameLen = strlen(name);
	fwrite(&nameLen,sizeof(unsigned char),1,f);
	fwrite(name,nameLen,1,f);
	fwrite(&xDim,sizeof(int),1,f);
	fwrite(&yDim,sizeof(int),1,f);
	fwrite(&zDim,sizeof(int),1,f);
	fwrite(&positionX,sizeof(float),1,f);
	fwrite(&positionY,sizeof(float),1,f);
	fwrite(&numCells,sizeof(int),1,f);
	fwrite(&signalLength,sizeof(int),1,f);
	for (int i=0;i<numCells;i++) cellArray[i]->save(f,lossy,forceOrder);
	fclose(f);
}

bool CFremenGrid::load(const char* filename)
{
	int ret = 0;
	signalLength = 0;
	FILE* f=fopen(filename,"r");
	if (f == NULL){
		printf("FrOctomap %s not found, aborting load.\n",filename);
		return false;
	}
	printf("Loading FrOctomap %s.\n",filename);
	for (int i=0;i<numCells;i++){
		 free(cellArray[i]);
		 //fprintf(stdout,"Cells %i %ld %d\n",i,sizeof(CFrelement*),signalLength);
	}
	free(cellArray);
	unsigned char nameLen = 0;
	ret = fread(&nameLen,sizeof(unsigned char),1,f);
	ret = fread(name,nameLen,1,f);
	name[nameLen] = 0;
	ret = fread(&xDim,sizeof(int),1,f);
	ret = fread(&yDim,sizeof(int),1,f);
	ret = fread(&zDim,sizeof(int),1,f);
	ret = fread(&positionX,sizeof(float),1,f);
	ret = fread(&positionY,sizeof(float),1,f);
	ret = fread(&numCells,sizeof(unsigned int),1,f);
	ret = fread(&signalLength,sizeof(unsigned int),1,f);
	cellArray = (CFrelement**) malloc(numCells*sizeof(CFrelement*));
	for (int i=0;i<numCells;i++) cellArray[i] = new CFrelement();
	for (int i=0;i<numCells;i++){
		cellArray[i]->load(f);
		cellArray[i]->signalLength = signalLength;
	}
	printf("FrOctomap %s loaded: name %s with %i: %ix%ix%i cells and %i observations.\n",filename,name,numCells,xDim,yDim,zDim,signalLength);
	fclose(f);
	return true;
}

void CFremenGrid::print(bool verbose)
{
	for (int i = 0;i<numCells;i++){
		if (cellArray[i]->order > 0 || cellArray[i]->outliers > 0){
			printf("Cell: %i ",i);
			cellArray[i]->print(verbose);
		}
	}
}

void CFremenGrid::reconstruct()
{
	unsigned char *reconstructed = (unsigned char*) malloc(signalLength*2);
	plan->prepare(signalLength);
	for (int i = 0;i<numCells;i++){
		if (cellArray[i]->order > 0 || cellArray[i]->outliers > 0){
			printf("Cell %i: ",i);
			cellArray[i]->reconstruct(reconstructed,plan);
			for (int j=0;j<signalLength;j++) printf("%i ",reconstructed[j]);
			printf("\n");
		}
	}
}

void CFremenGrid::reconstruct(int number,unsigned char *reconstructed)
{
	cellArray[number]->reconstruct(reconstructed,plan);
}

void CFremenGrid::add(unsigned int index,unsigned char value)
{
	cellArray[index]->add(value);
}

unsigned char CFremenGrid::retrieve(unsigned int index,unsigned int timeStamp)
{
	return cellArray[index]->retrieve(timeStamp);
}

float CFremenGrid::estimate(unsigned int index,unsigned int timeStamp)
{
	return cellArray[index]->estimate(timeStamp);
}

float CFremenGrid::fineEstimate(unsigned int index,float timeStamp)
{
	return cellArray[index]->fineEstimate(timeStamp);
}
