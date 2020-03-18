/***********************************************************************************
/* Create a new Phaser Game on window load
/***********************************************************************************/

window.onload = function () {
	var game = new Phaser.Game(1280,720, Phaser.CANVAS, 'game');
	
	game.state.add('Main', App.Main);
	game.state.start('Main');
};

/***********************************************************************************
/* Main program
/***********************************************************************************/

var App = {};

App.Main = function(game){
	this.STATE_INIT = 1;
	this.STATE_START = 2;
	this.STATE_PLAY = 3;
	this.STATE_GAMEOVER = 4;
	
	this.OBS_DISTANCE = 100;
}


var Obs_array=[];
//var previous_po=[];

var velocity=5;
//var theta_array=[];
App.Main.prototype = {

	preload : function(){
		this.game.load.image('imgTarget', 'assets/img_target.png');
		this.game.load.image('vehicle','assets/vehicle.png');
		this.game.load.image('obstacle','assets/obs.png');

this.game.load.image('border','assets/tp.png');
		this.game.load.spritesheet('imgButtons', 'assets/img_buttons.png', 110, 40, 3);
		
		this.game.load.image('imgTarget', 'assets/img_target.png');
		this.game.load.image('imgGround', 'assets/img_ground.png');
		this.game.load.image('imgPause', 'assets/img_pause.png');
		this.game.load.image('imgLogo', 'assets/img_logo.png');
		
		this.load.bitmapFont('fnt_chars_black', 'assets/fnt_chars_black.png', 'assets/fnt_chars_black.fnt');
		this.load.bitmapFont('fnt_digits_blue', 'assets/fnt_digits_blue.png', 'assets/fnt_digits_blue.fnt');
		this.load.bitmapFont('fnt_digits_green', 'assets/fnt_digits_green.png', 'assets/fnt_digits_green.fnt');
		this.load.bitmapFont('fnt_digits_red', 'assets/fnt_digits_red.png', 'assets/fnt_digits_red.fnt');
	},
	
	create : function(){
		// set scale mode to cover the entire screen
		this.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
		// set a blue color for the background of the stage
		this.game.stage.backgroundColor = '#ffffff';
		
		this.add.image(0,290,'border');
		this.add.image(0,500,'border');
		
		// create a new Genetic Algorithm with a population of 10 units which will be evolving by using 4 top units
		this.GA = new GeneticAlgorithm(20, 4);
		
		this.VehicleGroup = this.game.add.group();
		for (var i = 0; i < this.GA.max_units; i++){
			this.VehicleGroup.add(new vehicle(this.game, 50,400, i));
			//theta_array.push(0);
		}
	
		// this.ObsGroup = this.game.add.group();		
		// for (var i = 0; i < 4; i++){
		// 	new TreeGroup(this.game, this.ObsGroup, i);
		// }
		
		// create a Target Point sprite
		// this.TargetPoint = this.game.add.sprite(50, 400, 'imgTarget');
		// this.TargetPoint.anchor.setTo(0.5);
		
		////------add tween-------
		//this.obs_Tweens=this.add.tween();
		//var obs_Tweens=this.game.TweenManager();


		var obs01=this.add.sprite(100,300,'obstacle');
		var obs02=this.add.sprite(150,500,'obstacle');
		var obs03=this.add.sprite(200,300,'obstacle');
		var obs04=this.add.sprite(250,500,'obstacle');
		var obs05=this.add.sprite(300,300,'obstacle');
		var obs06=this.add.sprite(350,500,'obstacle');
		var obs07=this.add.sprite(400,300,'obstacle');
		var obs08=this.add.sprite(450,500,'obstacle');
		var obs09=this.add.sprite(500,300,'obstacle');
		var obs10=this.add.sprite(550,500,'obstacle');
		var obs11=this.add.sprite(600,300,'obstacle');
		var obs12=this.add.sprite(650,500,'obstacle');
		var obs13=this.add.sprite(700,300,'obstacle');
		var obs14=this.add.sprite(750,500,'obstacle');
		var obs15=this.add.sprite(800,300,'obstacle');
		var obs16=this.add.sprite(850,500,'obstacle');
		var obs17=this.add.sprite(900,300,'obstacle');
		var obs28=this.add.sprite(50,500,'obstacle');

		// var obs18=this.add.sprite(800,300,'obstacle');
		// var obs19=this.add.sprite(800,350,'obstacle');
		// var obs20=this.add.sprite(800,400,'obstacle');
		// var obs21=this.add.sprite(800,450,'obstacle');
		// var obs22=this.add.sprite(800,500,'obstacle');

		// var obs23=this.add.sprite(800,450,'obstacle');
		// var obs24=this.add.sprite(800,500,'obstacle');
		// var obs25=this.add.sprite(800,475,'obstacle');
		// var obs26=this.add.sprite(800,425,'obstacle');
		// var obs27=this.add.sprite(800,550,'obstacle');


		var tween_duration=4000;
		this.game.add.tween(obs01).to({y:500},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);
		this.game.add.tween(obs02).to({y:300},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);	
		this.game.add.tween(obs03).to({y:500},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs04).to({y:300},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs05).to({y:500},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs06).to({y:300},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs07).to({y:500},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs08).to({y:300},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs09).to({y:500},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs10).to({y:300},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs11).to({y:500},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs12).to({y:300},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs13).to({y:500},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs14).to({y:300},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs15).to({y:500},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs16).to({y:300},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs17).to({y:500},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);		
		this.game.add.tween(obs28).to({y:300},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,true);


		// this.game.add.tween(obs18).to({x:50},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,false);		
		// this.game.add.tween(obs19).to({x:50},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,false);		
		// this.game.add.tween(obs20).to({x:50},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,false);		
		// this.game.add.tween(obs21).to({x:50},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,false);		
		// this.game.add.tween(obs22).to({x:50},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,false);	

		// this.game.add.tween(obs23).to({x:50},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,false);		
		// this.game.add.tween(obs24).to({x:50},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,false);		
		// this.game.add.tween(obs25).to({x:50},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,false);		
		// this.game.add.tween(obs26).to({x:50},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,false);		
		// this.game.add.tween(obs27).to({x:50},tween_duration,"Linear",true,this.game.rnd.integerInRange(100, 2000),-1,false);
		
		Obs_array.push(obs01);
		Obs_array.push(obs02);
		Obs_array.push(obs03);
		Obs_array.push(obs04);
		Obs_array.push(obs05);
		Obs_array.push(obs06);
		Obs_array.push(obs07);
		Obs_array.push(obs08);
		Obs_array.push(obs09);
		Obs_array.push(obs10);
		Obs_array.push(obs11);
		Obs_array.push(obs12);
		Obs_array.push(obs13);
		Obs_array.push(obs14);
		Obs_array.push(obs15);
		Obs_array.push(obs16);
		Obs_array.push(obs17);
		Obs_array.push(obs28);

		// Obs_array.push(obs18);
		// Obs_array.push(obs19);
		// Obs_array.push(obs20);
		// Obs_array.push(obs21);
		// Obs_array.push(obs22);

		// Obs_array.push(obs23);
		// Obs_array.push(obs24);
		// Obs_array.push(obs25);
		// Obs_array.push(obs26);
		// Obs_array.push(obs27);

		this.obs_array=Obs_array;

		////------menu bar------
		//create a scrolling Ground object
		// this.Ground = this.game.add.tileSprite(0, this.game.height-100, this.game.width-370, 100, 'imgGround');
		// this.Ground.autoScroll(-200, 0);
		
		// create a BitmapData image for drawing head-up display (HUD) on it
		this.bmdStatus = this.game.make.bitmapData(370, this.game.height);
		this.bmdStatus.addToWorld(this.game.width - this.bmdStatus.width, 0);
		
		// create text objects displayed in the HUD header
		new Text(this.game, 1047, 10, "In1  In2  Out", "right", "fnt_chars_black"); // Input 1 | Input 2 | Output
		this.txtPopulationPrev = new Text(this.game, 1190, 10, "", "right", "fnt_chars_black"); // No. of the previous population
		this.txtPopulationCurr = new Text(this.game, 1270, 10, "", "right", "fnt_chars_black"); // No. of the current population
		
		//this.txtDist=new Text(this.game,200,50,"min_dist=", "left", "fnt_chars_black");
		// create text objects for each bird to show their info on the HUD
		this.txtStatusPrevGreen = [];	// array of green text objects to show info of top units from the previous population
		this.txtStatusPrevRed = [];		// array of red text objects to show info of weak units from the previous population
		this.txtStatusCurr = [];		// array of blue text objects to show info of all units from the current population
		
		for (var i=0; i<this.GA.max_units; i++){
			var y = 46 + i*50;
			
			new Text(this.game, 1110, y, "Fitness:\nScore:", "right", "fnt_chars_black")
			this.txtStatusPrevGreen.push(new Text(this.game, 1190, y, "", "right", "fnt_digits_green"));
			this.txtStatusPrevRed.push(new Text(this.game, 1190, y, "", "right", "fnt_digits_red"));
			this.txtStatusCurr.push(new Text(this.game, 1270, y, "", "right", "fnt_digits_blue"));
		}
		
		// create a text object displayed in the HUD footer to show info of the best unit ever born
		this.txtBestUnit = new Text(this.game, 1095, 580, "", "center", "fnt_chars_black");
		
		// create buttons
		this.btnRestart = this.game.add.button(920, 620, 'imgButtons', this.onRestartClick, this, 0, 0);
		this.btnMore = this.game.add.button(1040, 620, 'imgButtons', this.onMoreGamesClick, this, 2, 2);
		this.btnPause = this.game.add.button(1160, 620, 'imgButtons', this.onPauseClick, this, 1, 1);
		this.btnLogo = this.game.add.button(910, 680, 'imgLogo', this.onMoreGamesClick, this);
		
		// create game paused info
		this.sprPause = this.game.add.sprite(455, 360, 'imgPause');
		this.sprPause.anchor.setTo(0.5);
		this.sprPause.kill();
		
		// add an input listener that can help us return from being paused
		this.game.input.onDown.add(this.onResumeClick, this);
				
		// set initial App state
		this.state = this.STATE_INIT;
	},
	
	update : function(){		
		switch(this.state){
			case this.STATE_INIT: // init genetic algorithm
				this.GA.reset();
				this.GA.createPopulation();
				
				this.state = this.STATE_START;
				break;
				
			case this.STATE_START: // start/restart the game
				// update text objects
				this.txtPopulationPrev.text = "GEN "+(this.GA.iteration-1);
				this.txtPopulationCurr.text = "GEN "+(this.GA.iteration);
				
				this.txtBestUnit.text = 
					"The best unit was born in generation "+(this.GA.best_population)+":"+
					"\nFitness = "+this.GA.best_fitness.toFixed(2)+" / Score = " + this.GA.best_score;
				
				// reset score and distance
				this.score = 0;
				this.distance = 0;
				
				// // reset barriers
				// this.ObsGroup.forEach(function(obstacle){
				// 	obstacle.restart(700 + obstacle.index * this.OBS_DISTANCE);
				// }, this);
				
				// define pointer to the first barrier
				this.Obs_min_Dist=this.getNearestDist(50,400,0);
				// previous_po.push(50);
				// previous_po.push(400);
				
				//this.firstObs = this.ObsGroup.getAt(0);
				
				// // define pointer to the last barrier
				// this.lastObs = this.ObsGroup.getAt(thiss.ObsGroup.length-1);
				
				// // define pointer to the current target barrier
				// this.targetObs = this.firstObs;
				



				// start a new population of birds
				this.VehicleGroup.forEach(function(vehicle){
					vehicle.restart(this.GA.iteration);
					
					if (this.GA.Population[vehicle.index].isWinner){
						this.txtStatusPrevGreen[vehicle.index].text = vehicle.fitness_prev.toFixed(2)+"\n" + vehicle.score_prev;
						this.txtStatusPrevRed[vehicle.index].text = "";
					} else {
						this.txtStatusPrevGreen[vehicle.index].text = "";
						this.txtStatusPrevRed[vehicle.index].text = vehicle.fitness_prev.toFixed(2)+"\n" + vehicle.score_prev;
					}
				}, this);
							
				this.state = this.STATE_PLAY;
				break;
				
			case this.STATE_PLAY: // play Flappy Bird game by using genetic algorithm AI
				// update position of the target point
				// this.TargetPoint.x = this.targetObs.getGapX();
				// this.TargetPoint.y = this.targetObs.getGapY();
				
				var isNextTarget = false; // flag to know if we need to set the next target barrier
				

				this.VehicleGroup.forEachAlive(function(vehicle){
					//var x_previous=vehicle.x;

					this.Obs_min_Dist=this.getNearestDist(vehicle.x,vehicle.y,vehicle.rotation);
				
					//this.txtDist.text="dist="+this.Obs_min_Dist[0]+"    "+this.Obs_min_Dist[1]+"    "+this.Obs_min_Dist[2];
					//this.txtDist.text=vehicle.rotation;
					// calculate the current fitness and the score for this bird
					vehicle.fitness_curr = vehicle.x+100-Math.abs(vehicle.y-400);
					vehicle.score_curr = vehicle.x+100-Math.abs(vehicle.y-400);
					
					// check collision between a bird and the target barrier
					this.game.physics.arcade.collide(vehicle, this.targetObs, this.onDeath, null, this);
					var alive=1;
					for(var i=0;i<this.obs_array.length;i++)
					{
						var tp_dist=Math.sqrt((this.obs_array[i].x-vehicle.x)*(this.obs_array[i].x-vehicle.x)+(this.obs_array[i].y-vehicle.y)*(this.obs_array[i].y-vehicle.y));   
						if(tp_dist<10)
							alive=0;
					}

					
					if (alive==1){
						// check if a bird passed through the gap of the target barrier
						//if (vehicle.x > this.TargetPoint.x) isNextTarget = true;
						
						// check if a bird flies out of vertical bounds
						if (vehicle.y<300 || vehicle.y>500) this.onDeath(vehicle);
						//if (vehicle.x<x_previous ) this.onDeath(vehicle);
						if (vehicle.x<40 ) this.onDeath(vehicle);
						if (vehicle.x>=900) this.onDeath(vehicle);
						//if (vehicle.x<50) this.onDeath(vehicle);
						vehicle.score_curr = vehicle.x;
						// perform a proper action (flap yes/no) for this bird by activating its neural network
						this.GA.activateBrain(this.Obs_min_Dist, velocity, vehicle.rotation,vehicle);
					}
					else
						this.onDeath(vehicle);
						
				}, this);
				
				// // if any bird passed through the current target barrier then set the next target barrier
				// if (isNextTarget){
				// 	this.score++;

				// 	this.targetObs = this.getNextBarrier(this.targetObs.index);
				// }
				
				// // if the first barrier went out of the left bound then restart it on the right side
				// if (this.firstObs.getWorldX() < -this.firstObs.width){
				// 	this.firstObs.restart(this.lastObs.getWorldX() + this.OBS_DISTANCE);
					
				// 	this.firstObs = this.getNextObs(this.firstObs.index);
				// 	this.lastObs = this.getNextObs(this.lastObs.index);
				// }
				
				// // increase the travelled distance
				// this.distance = Math.abs(vehicle.x);
				
				this.drawStatus();				
				break;
				
			case this.STATE_GAMEOVER: // when all birds are killed evolve the population
				this.GA.evolvePopulation();
				this.GA.iteration++;
					
				this.state = this.STATE_START;
				break;
		}
	},
	
	drawStatus : function(){
		this.bmdStatus.fill(180, 180, 180); // clear bitmap data by filling it with a gray color
		this.bmdStatus.rect(0, 0, this.bmdStatus.width, 35, "#8e8e8e"); // draw the HUD header rect
			
		this.VehicleGroup.forEach(function(vehicle){
			var y = 85 + vehicle.index*50;
								
			this.bmdStatus.draw(vehicle, 25, y-25); // draw bird's image
			this.bmdStatus.rect(0, y, this.bmdStatus.width, 2, "#888"); // draw line separator
			
			if (vehicle.alive){
				var brain = this.GA.Population[vehicle.index].toJSON();
				var scale = this.GA.SCALE_FACTOR*0.02;
				
				this.bmdStatus.rect(62, y, 9, -(50 - brain.neurons[0].activation/scale), "#000088"); // input 1
				this.bmdStatus.rect(90, y, 9, brain.neurons[1].activation/scale, "#000088"); // input 2
				
				if (brain.neurons[brain.neurons.length-1].activation<0.5) this.bmdStatus.rect(118, y, 9, -20, "#880000"); // output: flap = no
				else this.bmdStatus.rect(118, y, 9, -40, "#008800"); // output: flap = yes
			}
			
			// draw bird's fitness and score
			this.txtStatusCurr[vehicle.index].setText(vehicle.fitness_curr.toFixed(2)+"\n" + vehicle.score_curr);
		}, this);
	},
	
	getNearestDist:function(x,y,rotate)
	{
		var dist_array=[];
		var min1=1000;
		var min2=1000;
		var min3=1000;
		var id1=0;
		var id2=0;
		var id3=0;
		for (var i = 0; i < this.obs_array.length; i++){
			var tp_dist=Math.sqrt((this.obs_array[i].x-x)*(this.obs_array[i].x-x)+(this.obs_array[i].y-y)*(this.obs_array[i].y-y));   
			var tp_angle=Math.atan2(this.obs_array[i].y-y,this.obs_array[i].x-x);
			var d_angle=rotate-tp_angle;
			if(d_angle<=Math.PI/2.0 && d_angle>Math.PI/6.0)
			{
				if(tp_dist<min1)
				{
					min1=tp_dist;
					id1=i;
				}
			}
			else if(d_angle<=Math.PI/6.0 && d_angle>=-Math.PI/6.0)
			{
				if(tp_dist<min2)
				{
					min2=tp_dist;
					id2=i;
				}
			}
			else if(d_angle<-Math.PI/6.0 && d_angle>=-Math.PI/2.0)
			{
				if(tp_dist<min3)
				{
					min3=tp_dist;
					id3=i;
				}
			}
		}

		dist_array.push(min1);
		dist_array.push(min2);
		dist_array.push(min3);
		dist_array.push(y-300);
		dist_array.push(500-y);
		return dist_array;

	},



	getNextObs : function(index){
		return this.ObsGroup.getAt((index + 1) % this.ObsGroup.length);
	},
	
	onDeath : function(vehicle){
		this.GA.Population[vehicle.index].fitness = vehicle.fitness_curr;
		this.GA.Population[vehicle.index].score = vehicle.score_curr;
					
		vehicle.death();
		if (this.VehicleGroup.countLiving() == 0) this.state = this.STATE_GAMEOVER;
	},
	
	onRestartClick : function(){
		this.state = this.STATE_INIT;
    },
	
	onMoreGamesClick : function(){
		window.open("http://www.askforgametask.com", "_blank");
	},
	
	onPauseClick : function(){
		this.game.paused = true;
		this.btnPause.input.reset();
		this.sprPause.revive();
    },
	
	onResumeClick : function(){
		if (this.game.paused){
			this.game.paused = false;
			this.btnPause.input.enabled = true;
			this.sprPause.kill();
		}
    }

    // launch:function(i){
    // 	if(i%2==0)
    // 	{

    // 	}
    // }
}

/***********************************************************************************
/* TreeGroup Class extends Phaser.Group
/***********************************************************************************/	
	
var TreeGroup = function(game, parent, index){
	Phaser.Group.call(this, game, parent);

	this.index = index;

	this.topTree = new Tree(this.game, 0); // create a top Tree object
	this.bottomTree = new Tree(this.game, 1); // create a bottom Tree object
	
	this.add(this.topTree); // add the top Tree to this group
	this.add(this.bottomTree); // add the bottom Tree to this group
};

TreeGroup.prototype = Object.create(Phaser.Group.prototype);
TreeGroup.prototype.constructor = TreeGroup;

TreeGroup.prototype.restart = function(x) {
	this.topTree.reset(0, 0);
	this.bottomTree.reset(0, this.topTree.height + 130);

	this.x = x;
	this.y = this.game.rnd.integerInRange(110-this.topTree.height, -20);

	this.setAll('body.velocity.x', -200);
};

// TreeGroup.prototype.getWorldX = function() {
// 	return this.topTree.world.x;
// };

// TreeGroup.prototype.getGapX = function() {
// 	return this.bottomTree.world.x + this.bottomTree.width;
// };

// TreeGroup.prototype.getGapY = function() {
// 	return this.bottomTree.world.y - 65;
// };


// var obs_tweens=function(game){
// 	Phaser.TweenManager.call(game);
// 	var obss=this.add.image();
// 	this.add({

// 	});
// };
// obs_tweens.prototype = Object.create(Phaser.TweenManager.prototype);
// obs_tweens.prototype.constructor = obs_tweens;

/***********************************************************************************
/* Tree Class extends Phaser.Sprite
/***********************************************************************************/

var Tree = function(game, frame) {
	Phaser.Sprite.call(this, game, 0, 0, 'obstacle', frame);
	
	//this.game.physics.arcade.enableBody(this);
	
	//this.body.allowGravity = false;
	this.body.immovable = false;
};

Tree.prototype = Object.create(Phaser.Sprite.prototype);
Tree.prototype.constructor = Tree;




/***********************************************************************************
/* Bird Class extends Phaser.Sprite
/***********************************************************************************/

var vehicle = function(game, x, y, index) {
	Phaser.Sprite.call(this, game, x, y, 'vehicle');
	   
	this.index = index;
	this.anchor.setTo(0.5);
	  
	// add flap animation and start to play it
	var i=index*2;
	// this.animations.add('flap', [i, i+1]);
	// this.animations.play('flap', 8, true);

	// enable physics on the bird
	this.game.physics.arcade.enableBody(this);
};

vehicle.prototype = Object.create(Phaser.Sprite.prototype);
vehicle.prototype.constructor = vehicle;

vehicle.prototype.restart = function(iteration){
	this.fitness_prev = (iteration == 1) ? 0 : this.fitness_curr;
	this.fitness_curr = 0;
	
	this.score_prev = (iteration == 1) ? 0: this.score_curr;
	this.score_curr = 0;
	
	this.body.rotation = 0;
	this.reset(50, 400);
};

vehicle.prototype.set_velocity = function(k){
	var dist=k*3;
this.body.x+=Math.cos(this.body.rotation)*dist;
this.body.y+=Math.sin(this.body.rotation)*dist;
//this.body.velocity.y = -400;
};

vehicle.prototype.set_dx = function(k){
 //     if (k<0.5)this.body.x-=2;
	// else if(k>=0.6)
	// 	this.body.x+=2;

	this.body.x+=(k-0.5)*5;

//this.body.y+=Math.sin(this.body.rotation)*dist;
//this.body.velocity.y = -400;
};

vehicle.prototype.set_dy= function(k){
	this.body.y+=(k-0.5)*5;
 //     if (k<0.5)this.body.y-=2;
	// else if(k>=0.6)
	// 	this.body.y+=2;
// 	var dist=(k-0.5)*5;
// this.body.y+=dist;
};


vehicle.prototype.set_rotate = function(k){
	this.body.rotation+=(k-0.5)*Math.PI/2.0 ;
};

vehicle.prototype.death = function(){
	this.body.rotation = 0;
	this.kill();

};

/***********************************************************************************
/* Text Class extends Phaser.BitmapText
/***********************************************************************************/

var Text = function(game, x, y, text, align, font){
	Phaser.BitmapText.call(this, game, x, y, font, text, 16);
	
	this.align = align;
	
	if (align == "right") this.anchor.setTo(1, 0);
	else this.anchor.setTo(0.5);
	
	this.game.add.existing(this);
};

Text.prototype = Object.create(Phaser.BitmapText.prototype);
Text.prototype.constructor = Text;

/***********************************************************************************
/* Obs Class extends Phaser.Tween
/***********************************************************************************/
// var obs = function(game, x, y, index) {
// 	Phaser.Tween.call(this, game, x, y, 'obstacle');


	   
// 	this.index = index;
// 	this.anchor.setTo(0.5);
	  
// 	// add flap animation and start to play it
// 	var i=index*2;
// 	// this.animations.add('flap', [i, i+1]);
// 	// this.animations.play('flap', 8, true);

// 	// enable physics on the bird
// 	this.game.physics.arcade.enableBody(this);
// };



