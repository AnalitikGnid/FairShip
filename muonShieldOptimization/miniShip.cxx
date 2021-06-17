#include "miniShip.h"
#include <math.h>
#include "vetoPoint.h"

#include "FairLogger.h"                 // for FairLogger, MESSAGE_ORIGIN
#include "FairVolume.h"
#include "FairGeoVolume.h"
#include "FairGeoNode.h"
#include "FairRootManager.h"
#include "FairGeoLoader.h"
#include "FairGeoInterface.h"
#include "FairGeoMedia.h"
#include "FairGeoBuilder.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"
#include "ShipDetectorList.h"
#include "ShipStack.h"

#include "TClonesArray.h"
#include "TVirtualMC.h"
#include "TGeoManager.h"
#include "TGeoBBox.h"
#include "TGeoBBox.h"
#include "TGeoCompositeShape.h"
#include "TGeoBoolNode.h"
#include "TGeoMaterial.h"
#include "TParticle.h"
#include "TROOT.h"
#include "TH1D.h"
#include "TH2D.h"
#include "TDatabasePDG.h"

#include <iostream>
using std::cout;
using std::endl;

miniShip::miniShip()
  : FairDetector("miniShip", kTRUE, kVETO),
    fTrackID(-1),
    fVolumeID(-1),
    fPos(),
    fMom(),
    fTime(-1.),
    fLength(-1.),
    fOnlyMuons(kFALSE),
    fSkipNeutrinos(kFALSE),
    fTargetMaterial("tungsten"),
    fInnerAbsMaterial("iron"),
    fOuterAbsMaterial("iron"),
    fConcreteShielding(kFALSE),
    fTargetL(1.),
    fAbsL(1.5),
    fField(1.0),
    fminiShipPointCollection(new TClonesArray("vetoPoint"))
{}

miniShip::~miniShip()
{
  if (fminiShipPointCollection) {
    fminiShipPointCollection->Delete();
    delete fminiShipPointCollection;
  }
}

Bool_t  miniShip::ProcessHits(FairVolume* vol)
{
  /** This method is called from the MC stepping */
  if ( gMC->IsTrackEntering() ) {
    fTime   = gMC->TrackTime() * 1.0e09;
    fLength = gMC->TrackLength();
    gMC->TrackPosition(fPos);
    gMC->TrackMomentum(fMom);
  }
  // Sum energy loss for all steps in the active volume
  if ( gMC->IsTrackExiting()    ||
       gMC->IsTrackStop()       ||
       gMC->IsTrackDisappeared()   ) {
   
    fTrackID  = gMC->GetStack()->GetCurrentTrackNumber();
    Int_t veto_uniqueId;
    gMC->CurrentVolID(veto_uniqueId);

    TParticle* p=gMC->GetStack()->GetCurrentTrack();
    Int_t pdgCode = p->GetPdgCode();
    TLorentzVector Pos;
    gMC->TrackPosition(Pos);
    TLorentzVector Mom;
    gMC->TrackMomentum(Mom);
    Double_t xmean = (fPos.X()+Pos.X())/2. ;
    Double_t ymean = (fPos.Y()+Pos.Y())/2. ;
    Double_t zmean = (fPos.Z()+Pos.Z())/2. ;
    AddHit(fTrackID, veto_uniqueId, TVector3(xmean, ymean,  zmean),
           TVector3(fMom.Px(), fMom.Py(), fMom.Pz()), fTime, fLength,
           0.,pdgCode,TVector3(Pos.X(), Pos.Y(), Pos.Z()),TVector3(Mom.Px(), Mom.Py(), Mom.Pz()) );
    // Increment number of veto det points in TParticle
    ShipStack* stack = (ShipStack*) gMC->GetStack();
    if (veto_uniqueId==13){stack->AddPoint(kVETO);}
  }
  return kTRUE;
}

// -----   Private method InitMedium 
Int_t miniShip::InitMedium(TString name) 
{
   static FairGeoLoader *geoLoad=FairGeoLoader::Instance();
   static FairGeoInterface *geoFace=geoLoad->getGeoInterface();
   static FairGeoMedia *media=geoFace->getMedia();
   static FairGeoBuilder *geoBuild=geoLoad->getGeoBuilder();

   FairGeoMedium *ShipMedium=media->getMedium(name);

   if (!ShipMedium)
     Fatal("InitMedium","Material %s not defined in media file.", name.Data());
   TGeoMedium* medium=gGeoManager->GetMedium(name);
   if (medium)
     return ShipMedium->getMediumIndex();
   return geoBuild->createMedium(ShipMedium);
}


void miniShip::Initialize()
{
  FairDetector::Initialize();
}

void miniShip::EndOfEvent()
{

  fminiShipPointCollection->Clear();

}

void miniShip::PreTrack(){
    gMC->TrackMomentum(fMom);
    if  ( (fMom.E()-fMom.M() )<EMax){
      gMC->StopTrack();
      return;
    }
    TParticle* p  = gMC->GetStack()->GetCurrentTrack();
    Int_t pdgCode = p->GetPdgCode();
// record statistics for neutrinos, electrons and photons
// add pi0 111 eta 221 eta' 331  omega 223 
    Int_t idabs = TMath::Abs(pdgCode);
    if (fSkipNeutrinos && (idabs==12 or idabs==14 or idabs == 16 )){gMC->StopTrack();}
    if (fOnlyMuons && idabs!=13 )                                  {gMC->StopTrack();}
}

void miniShip::PostTrack(){
// check if track reached sensitive plane, if not remove its points from collection
   Bool_t found = kFALSE;
   for (Int_t n=0;n<fminiShipPointCollection->GetEntries();n++) {
      vetoPoint* hit = (vetoPoint*)fminiShipPointCollection->At(n);
      Int_t detID = hit->GetDetectorID();
      if (detID==13 && hit->GetTrackID()==fTrackID){ found = kTRUE;}
   }
   if (!found){fminiShipPointCollection->Clear();}
}

void miniShip::FinishRun(){
}

void miniShip::ConstructGeometry()
{
   Double_t cm  = 1;       // cm
   Double_t m   = 100*cm;  //  m
   Double_t mm  = 0.1*cm;  //  mm
   Double_t kilogauss = 1.;
   Double_t tesla = 10 * kilogauss;

   TGeoVolume *top = gGeoManager->GetTopVolume();
   InitMedium(fTargetMaterial);
   TGeoMedium *TargetMaterial = gGeoManager->GetMedium(fTargetMaterial);
   InitMedium(fInnerAbsMaterial);
   TGeoMedium *InnerAbsMaterial  = gGeoManager->GetMedium(fInnerAbsMaterial);
   InitMedium(fOuterAbsMaterial);
   TGeoMedium *OuterAbsMaterial  = gGeoManager->GetMedium(fOuterAbsMaterial);
   InitMedium("vacuums");
   TGeoMedium *vac  = gGeoManager->GetMedium("vacuums");
   InitMedium("Concrete");
   TGeoMedium *concrete  = gGeoManager->GetMedium("Concrete");

   TGeoVolume* target        = gGeoManager->MakeBox("TargetArea",   TargetMaterial,  10.*cm,10.*cm,fTargetL/2*cm);
   target->SetLineColor(kRed);
   TGeoBBox* fullConcreteTarget  = new TGeoBBox("fullConcreteTarget",200.*cm,200.*cm,(fTargetL/2-0.01)*cm);
   TGeoSubtraction *subtractionT = new TGeoSubtraction("fullConcreteTarget","TargetArea");
   TGeoCompositeShape *csT       = new TGeoCompositeShape("outerTargetSubtr", subtractionT);
   TGeoVolume* outerTarget    = new TGeoVolume("outerTarget",csT, concrete);
   outerTarget->SetLineColor(kGray);
   AddSensitiveVolume(target);
   AddSensitiveVolume(outerTarget);

   TGeoVolume* innerAbsorber = gGeoManager->MakeBox("innerAbsorber",InnerAbsMaterial,10.*cm,10.*cm,fAbsL/2*cm);
   innerAbsorber->SetLineColor(kBlack);
   TGeoBBox* fullAbsorber  = new TGeoBBox("fullAbsorber",100.*cm,100.*cm,(fAbsL/2-0.01)*cm);
   TGeoSubtraction *subtraction = new TGeoSubtraction("fullAbsorber","innerAbsorber");
   TGeoCompositeShape *cs       = new TGeoCompositeShape("outerAbsorberSubtr", subtraction);
   TGeoVolume* outerAbsorber    = new TGeoVolume("outerAbsorber",cs, OuterAbsMaterial);
   outerAbsorber->SetLineColor(kGreen);
   AddSensitiveVolume(innerAbsorber);
   AddSensitiveVolume(outerAbsorber);

   TGeoBBox* fullConcreteAbsorber  = new TGeoBBox("fullConcreteAbsorber",400.*cm,400.*cm,(fAbsL/2-0.01)*cm);
   TGeoSubtraction *subtractionA = new TGeoSubtraction("fullConcreteAbsorber","fullAbsorber");
   TGeoCompositeShape *csA       = new TGeoCompositeShape("outerAbsorberSubtr", subtractionA);
   TGeoVolume* outerAbsorberConcrete    = new TGeoVolume("outerAbsorberConcrete",csA, concrete);
   outerAbsorberConcrete->SetLineColor(kGray);
   AddSensitiveVolume(outerAbsorberConcrete);

   if (fField > 0){
      TGeoUniformMagField *magField  = new TGeoUniformMagField(0.,fField*tesla,0.);
      outerAbsorber->SetField(magField);
      innerAbsorber->SetField(magField);
   }
   top->AddNode(target, 1, new TGeoTranslation(0, 0, 0));
   if (fConcreteShielding){top->AddNode(outerTarget, 11, new TGeoTranslation(0, 0, 0));}
   TGeoVolumeAssembly *muShield = new TGeoVolumeAssembly("MuonShieldArea");
   muShield->AddNode(innerAbsorber, 21,new TGeoTranslation(0, 0, (fTargetL/2+fAbsL/2+0.1)*cm));
   muShield->AddNode(outerAbsorber, 22,new TGeoTranslation(0, 0, (fTargetL/2+fAbsL/2+0.1)*cm));
   if (fConcreteShielding){muShield->AddNode(outerAbsorberConcrete, 23,new TGeoTranslation(0, 0, (fTargetL/2+fAbsL/2+0.1)*cm));}

   top->AddNode(muShield, 1, new TGeoTranslation(0, 0, 0));
   TGeoVolume *sensPlane = gGeoManager->MakeBox("sensPlane",vac,1000.*cm,1000.*cm,1.*mm);
   sensPlane->SetLineColor(kGreen);
   top->AddNode(sensPlane, 13, new TGeoTranslation(0, 0, (fTargetL/2+fAbsL+1)*cm));
   AddSensitiveVolume(sensPlane);
}


vetoPoint* miniShip::AddHit(Int_t trackID, Int_t detID,
                                      TVector3 pos, TVector3 mom,
                                      Double_t time, Double_t length,
                                      Double_t eLoss, Int_t pdgCode,TVector3 Lpos, TVector3 Lmom)
{
  TClonesArray& clref = *fminiShipPointCollection;
  Int_t size = clref.GetEntriesFast();
  return new(clref[size]) vetoPoint(trackID, detID, pos, mom,
         time, length, eLoss, pdgCode,Lpos,Lmom);
}

void miniShip::Register()
{

  FairRootManager::Instance()->Register("vetoPoint", "veto",
                                        fminiShipPointCollection, kTRUE);
}

TClonesArray* miniShip::GetCollection(Int_t iColl) const
{
  if (iColl == 0) { return fminiShipPointCollection; }
  else { return NULL; }
}
void miniShip::Reset()
{
  fminiShipPointCollection->Clear();
}

ClassImp(miniShip)
