CFLAGS =  -W -Wtraditional -Wmissing-prototypes -Wall 


# --- full object list
OBJ =        CConstraintHypergraph.o CTreeDecomposition.o \
				 CMCSTreeDecomposer.o CMinDegTreeDecomposer.o CMinFillTreeDecomposer.o \
				 CMinFillStaticTreeDecomposer.o \
				 CMinFillDegTreeDecomposer.o CTabuSearch.o 	 CDatabase.o \
				 CHypertreeNode.o CHypertreeDecomposer.o CHypertreeDecomposition.o \
				 COptTreeNode.o COptTreeDecomposer.o     COptTreeDecomposition.o


CCFILES =   CConstraintHypergraph.cpp CTreeDecomposition.cpp \
				 CMCSTreeDecomposer.cpp CMinDegTreeDecomposer.cpp  CMinFillTreeDecomposer.cpp \
				 CMinFillStaticTreeDecomposer.cpp \
				 CMinFillDegTreeDecomposer.cpp CTabuSearch.cpp 	 CDatabase.cpp \
				 CHypertreeNode.cpp CHypertreeDecomposer.cpp  CHypertreeDecomposition.cpp \
				 COptTreeNode.o COptTreeDecomposer.cpp    COptTreeDecomposition.cpp
HPPFILES = CConstraintHypergraph.h CTreeDecomposition.h STreeNode.h \
				 CMCSTreeDecomposer.h CMinDegTreeDecomposer.h CMinFillTreeDecomposer.h  \
				 CMinFillStaticTreeDecomposer.h \
				 CMinFillDegTreeDecomposer.h CTabuSearch.h cputime.h  SComp.h CDatabase.h \
				 CHypertreeNode.h CHypertreeDecomposer.h  CHypertreeDecomposition.h \
				 COptTreeNode.h COptTreeDecomposer.h   	  COptTreeDecomposition.h

# --------------------------------------------------------------
# Code generation
# --------------------------------------------------------------

.SUFFIXES: .cc .cpp 

.cpp.o:
	g++ -c  -O3 $<

# --------------------------------------------------------------
# The primary targets.
# --------------------------------------------------------------

libtreed.a: $(OBJ) $(HPPFILES)
	ar r libtreed.a $(OBJ) 
	ranlib libtreed.a


clean:
	rm -f *.o core *~
	rm libtreed.a

