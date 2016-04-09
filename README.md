# ED3D_project
Progetto finale di elaborazione dati tridimensionali - UNIPD - 2015-2016

Il metodo DrawLine pesca i valori per la creazione del laser dal file Configuration.xml.
Attenzione alla posizione di questo file, che per come è chiamato nel metodo DrawLine vuole stare in una cartella /data

Per ottenere l'immagine della scena, è necessario chiamare 
  get_pic(osg::ref_ptr<osg::Node> &_model, osg::Matrix &_trans, double _width, double _height)
Fornisce in output un cv::Mat 
es:	cv::Mat pippo = get_pic(loadedModel, trans, (double)width, (double)height);
