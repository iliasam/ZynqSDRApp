// Copyright (c) 2021-2023 John Seamons, ZL4VO/KF6VO

var prefs = {
   ext_name: 'prefs',    // NB: must match prefs.cpp:prefs_ext.name
   first_time: true,
   CMD1: 0,
   
   id: '_no_user_',
   pobj: null,
   default: { p:'default-p' },
   
   status_anim: false,
   status_timeout: -1,
};

function prefs_main()
{
	ext_switch_to_client(prefs.ext_name, prefs.first_time, prefs_recv);		// tell server to use us (again)
	if (!prefs.first_time)
		prefs_controls_setup();
	prefs.first_time = false;
}

function prefs_recv(data)
{
	var firstChars = arrayBufferToStringLen(data, 3);
	
	// process data sent from server/C by ext_send_msg_data()
	if (firstChars == "DAT") {
		var ba = new Uint8Array(data, 4);
		var cmd = ba[0];

		if (cmd == prefs.CMD1) {
			// do something ...
		} else {
			console.log('prefs_recv: DATA UNKNOWN cmd='+ cmd +' len='+ (ba.length-1));
		}
		return;
	}
	
	// process command sent from server/C by ext_send_msg() or ext_send_msg_encoded()
	var stringData = arrayBufferToString(data);
	var params = stringData.substring(4).split(" ");

	for (var i=0; i < params.length; i++) {
		var param = params[i].split("=");

		if (1 && param[0] != "keepalive") {
			if (isDefined(param[1]))
				console.log('prefs_recv: '+ param[0] +'='+ param[1]);
			else
				console.log('prefs_recv: '+ param[0]);
		}

		switch (param[0]) {

			case "ready":
				prefs_controls_setup();
				break;

			default:
				console.log('prefs_recv: UNKNOWN CMD '+ param[0]);
				break;
		}
	}
}

function prefs_controls_setup()
{
window.onstorage = function(e) {
   console.log('Key Changed: '+ e.key);
   console.log('New Value: '+ e.newValue);
};

   var controls_html =
      w3_divs('w3-text-css-yellow/w3-tspace-16',
         w3_div('w3-medium w3-text-aqua', '<b>User preferences</b>'),
         w3_div('id-prefs-container'),
         
         w3_div('w3-show-inline-block w3-hspace-16',
            w3_div('id-prefs-buttons w3-hide',
               w3_button('w3-margin-R-16', 'Export', 'prefs_export_btn_cb'),
               w3_button('w3-margin-R-16', 'Import', 'prefs_import_btn_cb')
            ),
            '<b>Status:</b> ' +
            w3_div('id-prefs-status w3-show-inline-block w3-snap-back')
         )
      );

   ext_panel_show(controls_html);
   //ext_set_controls_width_height(1024, 500);
   ext_set_controls_width_height(768, 250);
   prefs_status('yellow', 'waiting for local storage..');

	prefs_load(function() {
	   w3_innerHTML('id-prefs-container',
         w3_col_percent('',
            w3_input('', 'Prefs', 'prefs.pobj.p', prefs.pobj.p, 'prefs_p_cb', 'something'), 30
         )
      );
      w3_show('id-prefs-buttons');
	});
}

function prefs_p_cb(path, val)
{
	w3_string_cb(path, val);
	prefs_save();
}

function prefs_refresh_ui()
{
	w3_set_value('prefs.pobj.p', prefs.pobj.p);
}

function prefs_status(color, msg)
{
	var el = w3_el('id-prefs-status');
	
	if (prefs.status_anim) {
		kiwi_clearTimeout(prefs.status_timeout);
		//el.style.color = 'red';
		//el.innerHTML = 'CANCEL';
		w3_remove_then_add(el, 'w3-fade-out', 'w3-snap-back');
		setTimeout(function() {
			prefs.status_anim = false;
			prefs_status(color, msg);
		}, 1000);
		return;
	}
	
	el.style.color = color;
	el.innerHTML = msg;
	w3_remove_then_add(el, 'w3-snap-back', 'w3-fade-out');
	prefs.status_anim = true;
	prefs.status_timeout = setTimeout(function() {
		el.innerHTML = '';
		w3_remove_then_add(el, 'w3-fade-out', 'w3-snap-back');
		prefs.status_anim = false;
	}, 1500);
}

function prefs_export_btn_cb(path, val)
{
	console.log('prefs_export_btn_cb');
	prefs_save(function() {
		console.log('msg_send pref_export');
		// NB: use of 'pref' vs 'prefs' next line
		msg_send('SET pref_export id='+ encodeURIComponent(prefs.id) +' pref='+ encodeURIComponent(JSON.stringify(prefs.pobj)));
	});
	prefs_status('lime', 'preferences exported');
}

function prefs_import_btn_cb(path, val)
{
	console.log('prefs_import_btn_cb');
	var id = ident_user? ident_user : '_no_user_';
	// NB: use of 'pref' vs 'prefs' next line
	msg_send('SET pref_import id='+ encodeURIComponent(id));
}

function prefs_import_cb(p, ch)
{
	var s = decodeURIComponent(p);
	console.log('prefs_import_cb '+ s);
	if (s == 'null') {
		prefs_status('yellow', 'no preferences previously exported');
		return;
	} else {
		var obj = kiwi_JSON_parse('prefs_import_cb', s);
		if (obj) {
		   prefs.pobj = obj;
         console.log(obj);
         prefs_save();
         prefs_refresh_ui();
         prefs_status('lime', 'preferences successfully imported from RX'+ ch);
      }
	}
}
